from franky import *
import numpy as np

class ErrorLimit:
    def __init__(self, joint_threshold=0.01, joint_thresholds_individual=None, gripper_threshold=0.005):
        """
        初始化误差限制器
        joint_threshold: 统一关节角度阈值(弧度)，超过此值才认为需要移动
        joint_thresholds_individual: 每个关节的独立阈值列表[7个元素]，如果提供则覆盖统一阈值
        gripper_threshold: 夹爪位置阈值(米)，超过此值才认为需要移动
        """
        self._last_joint_list = [0.] * 7
        self._last_gripper_pos = 0.08
        self._joint_threshold = joint_threshold
        self._gripper_threshold = gripper_threshold
        
        # 如果提供了每个关节的独立阈值，则使用它；否则使用统一阈值
        if joint_thresholds_individual is not None:
            if len(joint_thresholds_individual) != 7:
                raise ValueError("joint_thresholds_individual必须包含7个元素")
            self._joint_thresholds_individual = joint_thresholds_individual.copy()
        else:
            self._joint_thresholds_individual = [joint_threshold] * 7
    
    def should_move_joint(self, current_joint_list) -> bool:
        """
        判断关节是否需要移动
        current_joint_list: 当前关节角度列表 [7个元素]
        返回: True表示需要移动，False表示不需要移动
        """
        if len(current_joint_list) != 7:
            raise ValueError("关节角度列表必须包含7个元素")
        
        # 计算每个关节与上次位置的差值，使用各自的阈值
        for i in range(7):
            diff = abs(current_joint_list[i] - self._last_joint_list[i])
            if diff > self._joint_thresholds_individual[i]:
                # 如果任何一个关节超出其对应阈值，更新位置并返回True
                self._last_joint_list = current_joint_list.copy()
                return True
        
        # 所有关节都在阈值范围内，不需要移动
        return False
    
    def should_move_gripper(self, current_gripper_pos) -> bool:
        """
        判断夹爪是否需要移动
        current_gripper_pos: 当前夹爪位置(米)
        返回: True表示需要移动，False表示不需要移动
        """
        diff = abs(current_gripper_pos - self._last_gripper_pos)
        if diff > self._gripper_threshold:
            # 超出阈值，更新位置并返回True
            self._last_gripper_pos = current_gripper_pos
            return True
        
        # 在阈值范围内，不需要移动
        return False
    
    def set_joint_threshold(self, threshold):
        """设置所有关节的统一阈值"""
        self._joint_threshold = threshold
        self._joint_thresholds_individual = [threshold] * 7
    
    def set_joint_threshold_individual(self, joint_index, threshold):
        """设置单个关节的阈值
        joint_index: 关节索引 (0-6)
        threshold: 阈值
        """
        if joint_index < 0 or joint_index > 6:
            raise ValueError("关节索引必须在0-6之间")
        self._joint_thresholds_individual[joint_index] = threshold
    
    def set_joint_thresholds_all_individual(self, thresholds):
        """设置所有关节的独立阈值
        thresholds: 长度为7的阈值列表
        """
        if len(thresholds) != 7:
            raise ValueError("阈值列表必须包含7个元素")
        self._joint_thresholds_individual = thresholds.copy()
    
    def set_gripper_threshold(self, threshold):
        """设置夹爪位置阈值"""
        self._gripper_threshold = threshold
    
    def get_joint_threshold(self):
        """获取当前统一关节角度阈值"""
        return self._joint_threshold
    
    def get_joint_threshold_individual(self, joint_index):
        """获取单个关节的阈值
        joint_index: 关节索引 (0-6)
        """
        if joint_index < 0 or joint_index > 6:
            raise ValueError("关节索引必须在0-6之间")
        return self._joint_thresholds_individual[joint_index]
    
    def get_joint_thresholds_all_individual(self):
        """获取所有关节的独立阈值列表"""
        return self._joint_thresholds_individual.copy()
    
    def get_gripper_threshold(self):
        """获取当前夹爪位置阈值"""
        return self._gripper_threshold
    
    def reset_last_positions(self, joint_list=None, gripper_pos=None):
        """重置上次记录的位置"""
        if joint_list is not None:
            self._last_joint_list = joint_list.copy()
        if gripper_pos is not None:
            self._last_gripper_pos = gripper_pos

    





class franka_controler():
    """""
    robot_ip: franka robot ip adress
    speed: max speed limit from 0.0 to 1.0.recommand to be 0.1-0.2
    impedence_behavior: trans robot to impedence control if impedence_behavior is not none. shape [7]
    """""
    def __init__(self,robot_ip,speed=0.1,impedence_behavior=None):
        self.robot = Robot(robot_ip)
        self.robot.recover_from_errors()
        self.robot.relative_dynamics_factor = speed

        self.gripper = Gripper(robot_ip)

        if impedence_behavior is not None:
            self.robot.set_joint_impedance(K_theta=impedence_behavior)
    
    def set_collision_behavior(self,lower_torque_threshold,upper_torque_threshold,lower_force_threshold,upper_force_threshold):
        """""
        lower_torque_threshold: 1*7 array
        upper_torque_threshold: 1*7 array
        lower_force_threshold: 1*6 array
        upper_force_threshold: 1*6 array
        """"" 
        self.robot.set_collision_behavior(
            lower_torque_threshold = lower_torque_threshold,
            upper_torque_threshold = upper_torque_threshold,
            lower_force_threshold = lower_force_threshold,
            upper_force_threshold = upper_force_threshold
            )

    def cartesian_move(self,target):
        """""
        it will block script until robot move to target pose
        target should be [x,y,z,qx,qy,qz,qw]
        """"" 
        motion = CartesianMotion(Affine(target[:3],target[3:]))
        
        self.robot.move(motion)
    
    def cartesian_move_asynchronous(self,target):
        """""
        the robot will move asynchronous
        target should be [x,y,z,qx,qy,qz,qw]
        """"" 
        motion = CartesianMotion(Affine(target[:3],target[3:]))
        self.robot.move(motion, asynchronous=True)

    def joint_move(self,target):
        """""
        it will block script until robot move to target pose
        target should be [q1--q7]
        """"" 
        motion = JointMotion(target)
        self.robot.move(motion)
    
    def joint_move_asynchronous(self,target):
        """""
        the robot will move asynchronous
        target should be [q1--q7]
        """"" 
        motion = JointMotion(target)
        self.robot.move(motion,asynchronous=True)

    def gripper_move(self,width,speed):
        self.gripper.move(width,speed)

    def gripper_move_asynchronous(self,width,speed):
        self.gripper.move_async(width,speed)
    
    def ee_pos_read(self):
        """""
        return the ee current pos, [x,y,z,qx,qy,qz,qw]
        """"" 
        r_pos = self.robot.current_cartesian_state.pose.end_effector_pose.translation
        r_ori = self.robot.current_cartesian_state.pose.end_effector_pose.quaternion
        print(self.robot.current_cartesian_state.pose.end_effector_pose)
        return np.concatenate((r_pos,r_ori)).flatten().tolist()
    
    def joint_pos_read(self):
        """""
        return the joint pos [7]
        """"" 
        return np.asanyarray(self.robot.current_joint_state.position).tolist()
    
    def joint_vel_read(self):
        """""
        return the joint velocity [7]
        """"" 
        return self.robot.current_joint_state.velocity
    
    def gripper_width_read(self) -> float:
        """""
        return the gripper width: float
        """"" 
        return self.gripper.width
    
