import sys
import time
import numpy as np
from math import copysign

import roslib
import rospy

from std_msgs.msg import String, Int8, Header
from geometry_msgs.msg import Twist, PoseStamped, Point, Quaternion, TransformStamped, PoseArray
from sensor_msgs.msg import CameraInfo, RegionOfInterest



class PBVS():
    def __init__(self):
        rospy.init_node('Position_Based_Visual_Servoing')
        rospy.on_shutdown(self.shutdown)
        self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=2)
        
        self.move_cmd = Twist()
        self.angular_dot = np.zeros((3,1), dtype=np.float32) # 3x1 angular velocity vector
        self.linear_dot = np.zeros((3,1), dtype=np.float32) # 3x1 linear velocity vector

        self.des_pos = Point()
        self.cur_pos = Point()
        self.update_velocity()
        self.update()

    def getCameraVelocity(self):
        error = np.zeros((3,1), dtype=np.float32)
        error[0] = self.cur_pos.x - self.des_pos.x
        error[1] = self.cur_pos.y - self.des_pos.y
        error[2] = self.cur_pos.z - self.des_pos.z
        print("Error:", error.T)
        velocity_camera_camera_frame = np.zeros((6,1), dtype = np.float32)
        return velocity_camera_camera_frame

    def visualServo(self):
        rx = 1
        ry = 1
        rz = 1
        sd = [[0, -rz, ry],
              [rz, 0, -rx],
              [-ry, rx, 0]]
        sd = np.array(sd) # skew symmetric matrix that defines camera location relative to base
        velocity_camera_camera_frame = self.getCameraVelocity()
        mat = np.zeros((6,6), dtype = np.float32)
        rot_camera_base = np.identity(3)
        for i in range(3):
            for j in range(3):
                mat[i][j] = rot_camera_base[i][j]
                mat[i][j+3] = sd[i][j]
                mat[i+3][j+3] = rot_camera_base[i][j]
        velocity_base_base_frame = np.dot(mat,velocity_camera_camera_frame)

        # rot_base_world = np.array([[1, 0, 0],
        #                            [0, 0,-1],
        #                            [0, 1, 0]])
        # J = np.zeros((6,6), dtype=np.float32) # Jacobian matrix
        # for i in range(3):
        #     for j in range(3):
        #         J[i][j] = rot_base_world[i][j]
        #         J[i+3][j+3] = rot_base_world[i][j]

        # velocity_base_world_frame =  np.dot(J,velocity_base_base_frame)
        
        # J_inv = np.linalg.pinv(J)
        # velocity_arr = np.dot(J_inv,velocity_base_world_frame)
        
        velocity_arr = velocity_base_base_frame
        self.linear_dot = velocity_arr[0:3]
        self.angular_dot = velocity_arr[3:6]
        self.update_velocity()

    def update_velocity(self):
        self.move_cmd.linear.x = self.linear_dot[0]
        self.move_cmd.linear.y = self.linear_dot[1]
        self.move_cmd.linear.z = self.linear_dot[2]
        self.move_cmd.angular.x = self.angular_dot[0]
        self.move_cmd.angular.y = self.angular_dot[1]
        self.move_cmd.angular.z = self.angular_dot[2]

    def update(self):
        rospy.loginfo("Starting the Position based Visual Servoing...")
        rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            self.visualServo()
            self.vel_pub.publish(self.move_cmd)
            rate.sleep()
        #rospy.spin()
    
    def shutdown(self):
        rospy.loginfo("Stopping the Position based Visual Servoing...")
        self.vel_pub.publish(Twist())
        rospy.sleep(1)


if __name__=='__main__':
    try:
        pbvs = PBVS()
        # rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Terminating Position based Visual Servo")