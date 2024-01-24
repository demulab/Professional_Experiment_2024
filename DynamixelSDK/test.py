import rclpy
from rclpy.node import Node
from dynamixel_sdk_custom_interfaces.msg import SetPosition
import numpy as np
import warnings
import time
from geometry_msgs.msg import PoseArray, Pose
warnings.simplefilter('ignore')

class OpenManipulatorX(Node):
    def __init__(self):
        super().__init__('OpenManipulatorX_node') 
        self.L1 = 0.077
        self.L2 = 0.13
        self.L3 = 0.124
        self.L4 = 0.126
        self.L5 = 0.024
        self.L6 = 0.128
        self.OPEN_DEGREE = -50.0
        self.CLOSE_DEGREE = 0.0
        self.ORIGIN = 2048
        self.ID = [11, 12, 13, 14, 15]
        self.DT = 1.0
        self.joint_angles = [0, 0, 0, 0, 0] 
        self.publisher = self.create_publisher(SetPosition, '/set_position', 10)
        self.create_subscription(Pose, '/aruco_poses_from_world', self.pose_callback, 10)

    def pose_callback(self, msg):
        self.marker_pose = Pose()
        bias_x = 0.04
        bias_y = 0.0
        bias_z = 0.02
        self.marker_pose.position.x = msg.position.x + bias_x
        self.marker_pose.position.y = msg.position.y + bias_y
        self.marker_pose.position.z = msg.position.z + bias_z
        print(self.marker_pose.position)
        
    def move(self, eef_pose, velocity = 100, acceleration = 10):
        """
        OpenManipulator-Xを目標姿勢に動かす.

        引数
        eef_pose     : エンドエフェクタの目標姿勢 [x(m), y(m), z(m), pitch(degree)]
        velocity     : アームの各関節モータの最大速度
        acceleration : アームの各関節モータの最大加速度
        """
        self.inverseKinematics(eef_pose)
        for i in range(4):
            self.revolution(self.ID[i], self.joint_angles[i], velocity, acceleration)
        time.sleep(self.DT)

    def initPose(self, velocity = 50, acceleration = 5):
        """
        OpenManipulator-Xを初期姿勢に動かす.

        引数
        velocity     : アームの各関節モータの最大速度
        acceleration : アームの各関節モータの最大加速度
        """
        self.joint_angles = [0, 0, 0, 0, 0]
        time.sleep(self.DT)
        for i in range(5):
            self.revolution(self.ID[i], self.joint_angles[i], velocity, acceleration)
        time.sleep(self.DT) 

    def openGripper(self, velocity = 100, acceleration = 20):
        """
        gripperを開く.

        引数
        velocity     : gripperモータの最大速度
        acceleration : gripperモータの最大加速度
        """
        self.revolution(self.ID[4], self.OPEN_DEGREE, velocity, acceleration)
        time.sleep(self.DT)

    def closeGripper(self, velocity = 100, acceleration = 20):
        """
        gripperを閉じる.

        引数
        velocity     : gripperモータの最大速度
        acceleration : gripperモータの最大加速度
        """
        self.revolution(self.ID[4], self.CLOSE_DEGREE, velocity, acceleration)
        time.sleep(self.DT)

    def revolution(self, motor_id, degree, velocity, acceleration):
        """
        指定したIDのモータを位置＋速度台形制御で回転させる.

        引数
        motor_id     : モータのID
        degree       : モータの目標角度[deg]
        velocity     : モータの最大速度
        acceleration : モータの最大加速度
        """
        msg = SetPosition()
        msg.id = motor_id
        msg.position = int(self.ORIGIN + degree / 0.088)
        msg.velocity = int(velocity)
        msg.acceleration = int(acceleration)
        self.publisher.publish(msg)

    def inverseKinematics(self, eef_pose):
        """
        OpenManipulatar-Xの逆運動学を解く.

        引数
        eef_pose     : エンドエフェクタの目標姿勢 [x(m), y(m), z(m), pitch(degree)]
        """
        theta_1 = np.arctan(eef_pose[1] / eef_pose[0])
        h = eef_pose[2] - self.L1 + self.L4 * np.sin(np.deg2rad(eef_pose[3]))
        k = np.hypot(eef_pose[0], eef_pose[1]) - self.L4 * np.cos(np.deg2rad(eef_pose[3]))
        phi = np.arctan2(h, k)
        psi = np.arctan2(self.L5, self.L6)
        v = np.hypot(h, k)
        alpha = np.arccos((self.L2**2 + v**2 - self.L3**2) / (2.0 * self.L2 * v))
        theta_2 = np.pi / 2.0 - alpha - phi - psi
        beta = np.arccos(
                (self.L2**2 + self.L3**2 -v**2) / (2.0 * self.L2 * self.L3)
                )
        theta_3 = np.pi - beta - np.pi / 2.0 + psi
        theta_4 = np.deg2rad(eef_pose[3]) - (theta_2 + theta_3)
        if self.checkAngleValid(theta_1, theta_2, theta_3, theta_4) == True:
            self.joint_angles = [np.rad2deg(theta_1) , np.rad2deg(theta_2), np.rad2deg(theta_3), np.rad2deg(theta_4), 0.0]
        else:
            print('\033[31m'+'OpenMANIPULATOR-Xでは実現できないeef_poseが設定されました。'+'\033[0m')

    def checkAngleValid(self, angle1, angle2, angle3, angle4, show=False):
        """
        inverseKinematicsで計算された各関節角度にnanが含まれるか確認する.

        引数
        angle1~4 : モータの目標関節角度[rad]
        
        返り値
        True     : 計算が正常に行われてnanが含まれていない.
        False    : nanが含まれている.
        """ 
        if show == True:
            print('theta_1 :', np.rad2deg(angle1))
            print('theta_2 :', np.rad2deg(angle2))
            print('theta_3 :', np.rad2deg(angle3))
            print('theta_4 :', np.rad2deg(angle4))
        if(np.isnan(angle1)): return False
        if(np.isnan(angle2)): return False
        if(np.isnan(angle3)): return False
        if(np.isnan(angle4)): return False
        return True

def main(args=None):
    rclpy.init(args=args)
    open_manipulator_x = OpenManipulatorX()  
   
    open_manipulator_x.initPose()
    open_manipulator_x.openGripper() 
    while True:
        vel = 50
        accel = 10
        open_manipulator_x.move([0.1, 0.1, 0.1, 90.0], vel, accel)
        open_manipulator_x.move([0.1, 0.1, 0.05, 90.0], vel, accel)
        open_manipulator_x.closeGripper()
        open_manipulator_x.move([0.1, -0.1, 0.1, 90.0], vel, accel)
        open_manipulator_x.move([0.1, -0.1, 0.05, 90.0], vel, accel)
        open_manipulator_x.openGripper()
 
        vel = 500
        accel = 70
        open_manipulator_x.move([0.1, 0.0, 0.2, 0.0], vel, accel)
        open_manipulator_x.move([0.1, -0.1, 0.2, 0.0], vel, accel)
        open_manipulator_x.move([0.25, 0.0, 0.15, 45.0], vel, accel)
        open_manipulator_x.move([0.1, -0.1, 0.2, -20.0], vel, accel)

                
    rclpy.shutdown()

if __name__ == '__main__':
    main()

"""
2023/11/14
KAWASUMI Ren
"""
