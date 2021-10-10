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
        self.theta_dot = np.zeros((3)) # 3x1 angular velocity vector
        self.linear_dot = np.zeros((3)) # 3x1 linear velocity vector

        self.des_pos = Point()
        self.cur_pos = Point()
        
    def update_velocity(self):
        self.move_cmd.linear.x = self.linear_dot[0]
        self.move_cmd.linear.y = self.linear_dot[1]
        self.move_cmd.linear.z = self.linear_dot[2]
        self.move_cmd.angular.x = self.theta_dot[0]
        self.move_cmd.angular.y = self.theta_dot[1]
        self.move_cmd.angular.z = self.theta_dot[2]

if __name__=='__main__':
    try:
        pbvs = PBVS()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Terminating Visual Servo")