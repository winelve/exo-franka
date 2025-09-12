import time
from threading import Event, Lock, Thread
from typing import Protocol, Sequence

import numpy as np

from dynamixel_sdk.group_sync_read import GroupSyncRead
from dynamixel_sdk.group_sync_write import GroupSyncWrite
from dynamixel_sdk.packet_handler import PacketHandler
from dynamixel_sdk.port_handler import PortHandler
from dynamixel_sdk.robotis_def import (
    COMM_SUCCESS,
    DXL_HIBYTE,
    DXL_HIWORD,
    DXL_LOBYTE,
    DXL_LOWORD,
)

# Constants
ADDR_OPERATING_MODE = 11  # X-series: 0=Current, 1=Velocity, 3=Position, 5=Current-based Position, 16=PWM
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_CURRENT = 102
LEN_GOAL_CURRENT = 2
ADDR_GOAL_POSITION = 116
LEN_GOAL_POSITION = 4
ADDR_PRESENT_POSITION = 132  # Fix: Present Position address
ADDR_PRESENT_CURRENT = 126
LEN_PRESENT_CURRENT = 2
LEN_PRESENT_POSITION = 4
TORQUE_ENABLE = 1
TORQUE_DISABLE = 0


class DynamixelDriverProtocol(Protocol):
    def set_joints(self, joint_angles: Sequence[float]):
        """Set the joint angles for the Dynamixel servos.

        Args:
            joint_angles (Sequence[float]): A list of joint angles.
        """
        ...

    def torque_enabled(self) -> bool:
        """Check if torque is enabled for the Dynamixel servos.

        Returns:
            bool: True if torque is enabled, False if it is disabled.
        """
        ...

    def set_torque_mode(self, enable: bool):
        """Set the torque mode for the Dynamixel servos.

        Args:
            enable (bool): True to enable torque, False to disable.
        """
        ...

    def get_joints(self) -> np.ndarray:
        """Get the current joint angles in radians.

        Returns:
            np.ndarray: An array of joint angles.
        """
        ...

    def close(self):
        """Close the driver."""


class DynamixelDriver(DynamixelDriverProtocol):
    def __init__(
        self,
        ids: Sequence[int],
        port: str = "/dev/ttyUSB0",
        baudrate: int = 57600,
        current_unit_mA_per_tick: float = 2.69,
        protocol_version: float = 2.0,
    ):
        """Initialize the DynamixelDriver class.

        Args:
            ids (Sequence[int]): A list of IDs for the Dynamixel servos.
            port (str): The USB port to connect to the arm.
            baudrate (int): The baudrate for communication.
            current_unit_mA_per_tick (float): Conversion factor from device tick to mA (X-series ≈ 2.69mA/LSB).
            protocol_version (float): Dynamixel protocol version (1.0 or 2.0). Default 2.0.
        """
        self._ids = ids
        self._joint_angles = None
        self._lock = Lock()
        self._current_unit_mA_per_tick = current_unit_mA_per_tick
        self._operating_mode = None  # Unknown until set/read

        # Initialize the port handler, packet handler, and group sync read/write
        self._portHandler = PortHandler(port)
        self._packetHandler = PacketHandler(protocol_version)

        # Group readers/writers
        self._groupSyncRead = GroupSyncRead(
            self._portHandler,
            self._packetHandler,
            ADDR_PRESENT_POSITION,
            LEN_PRESENT_POSITION,
        )
        self._groupSyncWrite = GroupSyncWrite(
            self._portHandler,
            self._packetHandler,
            ADDR_GOAL_POSITION,
            LEN_GOAL_POSITION,
        )
        # Current (torque) control sync read/write
        self._groupSyncReadCurrent = GroupSyncRead(
            self._portHandler,
            self._packetHandler,
            ADDR_PRESENT_CURRENT,
            LEN_PRESENT_CURRENT,
        )
        self._groupSyncWriteCurrent = GroupSyncWrite(
            self._portHandler,
            self._packetHandler,
            ADDR_GOAL_CURRENT,
            LEN_GOAL_CURRENT,
        )

        # Open the port and set the baudrate
        if not self._portHandler.openPort():
            raise RuntimeError("Failed to open the port")

        if not self._portHandler.setBaudRate(baudrate):
            raise RuntimeError(f"Failed to change the baudrate, {baudrate}")

        # Add parameters for each Dynamixel servo to the group sync read
        for dxl_id in self._ids:
            if not self._groupSyncRead.addParam(dxl_id):
                raise RuntimeError(
                    f"Failed to add parameter for Dynamixel with ID {dxl_id}"
                )
            if not self._groupSyncReadCurrent.addParam(dxl_id):
                raise RuntimeError(
                    f"Failed to add current read parameter for Dynamixel with ID {dxl_id}"
                )

        # Disable torque for each Dynamixel servo
        self._torque_enabled = False
        try:
            self.set_torque_mode(self._torque_enabled)
        except Exception as e:
            print(f"port: {port}, {e}")

        # control the thread
        self._stop_thread = Event()

        self._start_reading_thread()

    def set_joints(self, joint_angles: Sequence[float]):
        if len(joint_angles) != len(self._ids):
            raise ValueError(
                "The length of joint_angles must match the number of servos"
            )
        if not self._torque_enabled:
            raise RuntimeError("Torque must be enabled to set joint angles")

        for dxl_id, angle in zip(self._ids, joint_angles):
            # Convert the angle to the appropriate value for the servo
            position_value = int(angle * 2048 / np.pi)

            # Allocate goal position value into byte array
            param_goal_position = [
                DXL_LOBYTE(DXL_LOWORD(position_value)),
                DXL_HIBYTE(DXL_LOWORD(position_value)),
                DXL_LOBYTE(DXL_HIWORD(position_value)),
                DXL_HIBYTE(DXL_HIWORD(position_value)),
            ]

            # Add goal position value to the Syncwrite parameter storage
            dxl_addparam_result = self._groupSyncWrite.addParam(
                dxl_id, param_goal_position
            )
            if not dxl_addparam_result:
                raise RuntimeError(
                    f"Failed to set joint angle for Dynamixel with ID {dxl_id}"
                )

        # Syncwrite goal position
        dxl_comm_result = self._groupSyncWrite.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            raise RuntimeError("Failed to syncwrite goal position")

        # Clear syncwrite parameter storage
        self._groupSyncWrite.clearParam()

    def torque_enabled(self) -> bool:
        return self._torque_enabled

    def set_operating_mode(self, mode: int):
        """Set operating mode for all motors.

        Note: Requires torque disabled. This method will handle disabling and
        restoring torque state automatically.
        """
        # If torque is enabled, temporarily disable it for mode change
        restore_torque = self._torque_enabled
        if restore_torque:
            self.set_torque_mode(False)
        with self._lock:
            for dxl_id in self._ids:
                dxl_comm_result, dxl_error = self._packetHandler.write1ByteTxRx(
                    self._portHandler, dxl_id, ADDR_OPERATING_MODE, mode
                )
                if dxl_comm_result != COMM_SUCCESS or dxl_error != 0:
                    err_comm = self._packetHandler.getTxRxResult(dxl_comm_result)
                    err_rx = self._packetHandler.getRxPacketError(dxl_error)
                    raise RuntimeError(
                        f"Failed to set operating mode for Dynamixel ID {dxl_id}: comm={dxl_comm_result}({err_comm}), err={dxl_error}({err_rx})"
                    )
        self._operating_mode = mode
        if restore_torque:
            self.set_torque_mode(True)

    def set_torque_mode(self, enable: bool):
        torque_value = TORQUE_ENABLE if enable else TORQUE_DISABLE
        with self._lock:
            for dxl_id in self._ids:
                dxl_comm_result, dxl_error = self._packetHandler.write1ByteTxRx(
                    self._portHandler, dxl_id, ADDR_TORQUE_ENABLE, torque_value
                )
                if dxl_comm_result != COMM_SUCCESS or dxl_error != 0:
                    print(dxl_comm_result)
                    print(dxl_error)
                    err_comm = self._packetHandler.getTxRxResult(dxl_comm_result)
                    err_rx = self._packetHandler.getRxPacketError(dxl_error)
                    raise RuntimeError(
                        f"Failed to set torque mode for Dynamixel with ID {dxl_id}: comm={dxl_comm_result}({err_comm}), err={dxl_error}({err_rx})"
                    )

        self._torque_enabled = enable

    def set_torque_for(self, ids: Sequence[int], enable: bool):
        """Enable/disable torque for selected motor IDs only. Others unchanged."""
        torque_value = TORQUE_ENABLE if enable else TORQUE_DISABLE
        with self._lock:
            for dxl_id in ids:
                if dxl_id not in self._ids:
                    continue
                dxl_comm_result, dxl_error = self._packetHandler.write1ByteTxRx(
                    self._portHandler, dxl_id, ADDR_TORQUE_ENABLE, torque_value
                )
                if dxl_comm_result != COMM_SUCCESS or dxl_error != 0:
                    err_comm = self._packetHandler.getTxRxResult(dxl_comm_result)
                    err_rx = self._packetHandler.getRxPacketError(dxl_error)
                    raise RuntimeError(
                        f"Failed to set torque for ID {dxl_id}: "
                        f"comm={dxl_comm_result}({err_comm}), err={dxl_error}({err_rx})"
                    )

    def _start_reading_thread(self):
        self._reading_thread = Thread(target=self._read_joint_angles)
        self._reading_thread.daemon = True
        self._reading_thread.start()

    def _read_joint_angles(self):
        # Continuously read joint angles and update the joint_angles array
        while not self._stop_thread.is_set():
            time.sleep(0.001)
            with self._lock:
                _joint_angles = np.zeros(len(self._ids), dtype=int)
                dxl_comm_result = self._groupSyncRead.txRxPacket()
                if dxl_comm_result != COMM_SUCCESS:
                    print(f"warning, comm failed: {dxl_comm_result}")
                    continue
                for i, dxl_id in enumerate(self._ids):
                    if self._groupSyncRead.isAvailable(
                        dxl_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION
                    ):
                        angle = self._groupSyncRead.getData(
                            dxl_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION
                        )
                        angle = np.int32(np.uint32(angle))
                        _joint_angles[i] = angle
                    else:
                        raise RuntimeError(
                            f"Failed to get joint angles for Dynamixel with ID {dxl_id}"
                        )
                self._joint_angles = _joint_angles

    def get_joints(self) -> np.ndarray:
        # Return a copy of the joint_angles array to avoid race conditions
        while self._joint_angles is None:
            time.sleep(0.1)
        with self._lock:
            _j = self._joint_angles.copy()
        return _j / 2048.0 * np.pi

    # ----- Current (Torque) control helpers -----
    def set_currents(self, currents_mA: Sequence[float]):
        """Set per-motor current (approximate torque) in milliamps.

        For X-series, device unit is ~2.69 mA/LSB (configurable). Values are signed.
        Requires torque enabled and operating mode 0 (Current) or 5 (Current-based Position).
        """
        if len(currents_mA) != len(self._ids):
            raise ValueError("Length of currents_mA must match number of servos")
        if not self._torque_enabled:
            raise RuntimeError("Torque must be enabled to set current")
        with self._lock:
            for dxl_id, mA in zip(self._ids, currents_mA):
                ticks = int(round(mA / self._current_unit_mA_per_tick))
                # Clamp to 16-bit signed range just in case
                if ticks < -32768:
                    ticks = -32768
                if ticks > 32767:
                    ticks = 32767
                u16 = ticks & 0xFFFF
                param_goal_current = [
                    DXL_LOBYTE(DXL_LOWORD(u16)),
                    DXL_HIBYTE(DXL_LOWORD(u16)),
                ]
                if not self._groupSyncWriteCurrent.addParam(dxl_id, param_goal_current):
                    raise RuntimeError(
                        f"Failed to set goal current for Dynamixel with ID {dxl_id}"
                    )
            dxl_comm_result = self._groupSyncWriteCurrent.txPacket()
            if dxl_comm_result != COMM_SUCCESS:
                raise RuntimeError("Failed to syncwrite goal current")
            self._groupSyncWriteCurrent.clearParam()

    def get_currents(self) -> np.ndarray:
        """Read present current for all motors, returned in milliamps (signed)."""
        with self._lock:
            dxl_comm_result = self._groupSyncReadCurrent.txRxPacket()
            if dxl_comm_result != COMM_SUCCESS:
                raise RuntimeError(f"Failed to syncread present current: {dxl_comm_result}")
            vals = np.zeros(len(self._ids), dtype=np.int16)
            for i, dxl_id in enumerate(self._ids):
                if self._groupSyncReadCurrent.isAvailable(
                    dxl_id, ADDR_PRESENT_CURRENT, LEN_PRESENT_CURRENT
                ):
                    raw = self._groupSyncReadCurrent.getData(
                        dxl_id, ADDR_PRESENT_CURRENT, LEN_PRESENT_CURRENT
                    )
                    # Convert to signed 16-bit
                    raw = np.int16(np.uint16(raw))
                    vals[i] = int(raw)
                else:
                    raise RuntimeError(
                        f"Failed to get present current for Dynamixel with ID {dxl_id}"
                    )
        return vals.astype(float) * self._current_unit_mA_per_tick

    def set_current_map(self, currents_by_id: dict[int, float], default: float | None = None):
        """Set currents using a {id: mA} mapping.

        If default is not None, unspecified IDs will be set to default. Otherwise
        only specified IDs are updated.
        """
        if default is None:
            # Only send params for specified IDs
            if not self._torque_enabled:
                raise RuntimeError("Torque must be enabled to set current")
            with self._lock:
                for dxl_id, mA in currents_by_id.items():
                    if dxl_id not in self._ids:
                        continue
                    ticks = int(round(mA / self._current_unit_mA_per_tick)) & 0xFFFF
                    param_goal_current = [
                        DXL_LOBYTE(DXL_LOWORD(ticks)),
                        DXL_HIBYTE(DXL_LOWORD(ticks)),
                    ]
                    if not self._groupSyncWriteCurrent.addParam(dxl_id, param_goal_current):
                        raise RuntimeError(f"Failed to set goal current for ID {dxl_id}")
                dxl_comm_result = self._groupSyncWriteCurrent.txPacket()
                if dxl_comm_result != COMM_SUCCESS:
                    raise RuntimeError("Failed to syncwrite goal current (map)")
                self._groupSyncWriteCurrent.clearParam()
        else:
            # Build full list aligned with self._ids
            currents = [currents_by_id.get(dxl_id, default) for dxl_id in self._ids]
            self.set_currents(currents)

    def close(self):
        self._stop_thread.set()
        self._reading_thread.join()
        self._portHandler.closePort()
