import sys
import time
import numpy as np

import roslib
import rospy

from std_msgs.msg import String, Int8, Header
from geometry_msgs.msg import Twist, PoseStamped, Point, Quaternion, TransformStamped, PoseArray
from sensor_msgs.msg import CameraInfo, RegionOfInterest

class PBVS():
    curr_pose = Point()
    des_pose = Point()

    distThreshold = 0.4
    
    KP = 0.008
    KD = 0.0004
    KI = 0.00005

    x_sum_error = 0
    y_sum_error = 0
    z_sum_error = 0
    
    x_prev_error = 0
    y_prev_error = 0
    z_prev_error = 0
    
    x_change = 1
    y_change = 1
    z_change = 1

    def __init__(self):
        rospy.init_node('PBVS')
        rospy.Subscriber('ObjectRecognition', PoseArray, self.getObjPos)
        vel_pub = rospy.Publisher('cmd_vel', Twist)

    def getObjPos(self, pos):
        des_pos = pos.pose.position
    
    def getRobPos(self, pos):
        curr_pos = pos.pose.position

    def update(self):
        pass

    def spin(self):
        rospy.loginfo("Initialising Visual Servo")
        
        while not rospy.is_shutdown():
            self.update()
        rospy.spin()

if __name__=='__main__':
    pbvs = PBVS()
    pbvs.spin()