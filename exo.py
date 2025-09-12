from dynamixel.driver import DynamixelDriver
import numpy as np
import time
from typing import List

import math

class ExoPosition:
    def __init__(self, port="COM5", baudrate=1000000):
        self.joint_ids = (8, 9, 10, 11, 12, 13, 14, 15)
        self.driver = DynamixelDriver(self.joint_ids, port=port, baudrate=baudrate)
        self.driver.set_operating_mode(0)
        self.driver.set_torque_mode(True)
        # self.driver.set_current_map({15:85},default=-100)
        self.driver.set_current_map({15: 70, 9:-60}, default=0)
        self.driver.set_torque_for([10,11,12,13,14], False)
        # 给的偏移量是：   0     -0.7854 ,0 ,-2.3562,0,1.5708, 0 
        self.offset = [3.14159,-3.14159,3.14159,1.5708,3.14159,3.14159,-0.7854, 0, 0]
        self.direction = [1,1,1,1,1,1,1,1,1]

        self.gripper_range = (2.06,3.5195)
        self.gripper_target = (0,0.08)
        self.gripper_scale = (self.gripper_range[1] - self.gripper_range[0])/self.gripper_target[1]  
    
    def normalize_angle(self, angle):
        """
        将角度归一化到[-π, π]范围内
        """
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def getPosition(self):
        """
        return joint_pos shape(9): [h,g,f,e,d,c,b,a,0]
        """
        joint_np = self.driver.get_joints()
        # [h,g,f,e,d,c,b,a]
        joint_pos = list(joint_np)
     
        # 范围:(2.03099, 3.51895)
        # 映射:(0      , 0.08)
        joint_pos[-1] = (joint_pos[-1]-self.gripper_range[0]) / self.gripper_scale
        joint_pos.append(0)
        
        joint_pos = [joint_pos[i] + self.offset[i] for i in range(len(joint_pos))] 
        joint_pos = [joint_pos[i] * self.direction[i] for i in range(len(joint_pos))]
        
        # 对所有关节角度进行归一化（除了夹爪位置，假设最后一个是夹爪）
        for i in range(len(joint_pos)-2):  # 排除最后一个夹爪位置
            joint_pos[i] = self.normalize_angle(joint_pos[i])
        return joint_pos
    
    def getJointPos(self) -> List[float]:
        return self.getPosition()[0:7]
    
    def getGripPos(self) -> float:
        return self.getPosition()[-2]
    
    

if __name__ == "__main__":
    exo = ExoPosition()
    last_joints = np.zeros(8)
    while True:
        joints = exo.getPosition()
        joints = [round(float(j),5) for j in joints]
        print(f'max idx: {joints.index(max(joints))} ----> {joints}')

        # delta_joints = joints - last_joints

        # delta_joints = [round(j,3) for j in list(delta_joints)]
        # print(f'max idx: {delta_joints.index(max(delta_joints))} ----> {delta_joints}')

        # last_joints = joints
        time.sleep(0.3)


