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
        rospy.init_node('Visual_Servoing')
        rospy.on_shutdown(self.shutdown)
        self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.move_cmd = Twist()

        # The maximum distance a target can be from the robot for us to track
        self.max_z = rospy.get_param("~max_z", 10.0)
        

        # The goal distance (in meters) to keep between the robot and the marker
        self.goal_z = rospy.get_param("~goal_z", 1.0)
        # The goal distance (in meters) to keep between the robot and the marker
        self.goal_x = rospy.get_param("~goal_x", 320)


        # How far away from the goal distance (in meters) before the robot reacts
        self.z_threshold = rospy.get_param("~z_threshold", 0.5)
        # How far away from being centered (x displacement) of the object before the robot reacts (units are meters)
        self.x_threshold = rospy.get_param("~x_threshold", 100)
        

        # How much do we weight the goal distance (z) when making a movement
        self.z_scale = rospy.get_param("~z_scale", 0.05)
        # How much do we weight x-displacement when making a movement        
        self.x_scale = rospy.get_param("~x_scale", 0.001)


        # The maximum rotation speed in radians per second
        self.max_angular_speed = rospy.get_param("~max_angular_speed", 0.8)
        # The minimum rotation speed in radians per second
        self.min_angular_speed = rospy.get_param("~min_angular_speed", 0.5)


        # The max linear speed in meters per second
        self.max_linear_speed = rospy.get_param("~max_linear_speed", 0.3)
        # The minimum linear speed in meters per second
        self.min_linear_speed = rospy.get_param("~min_linear_speed", 0.1)


        # Set flag to indicate when the AR marker is visible
        self.target_visible = False

         # Wait for the ObjectRecognition topic to become available
        rospy.loginfo("Waiting for object_pose topic...")
        #rospy.wait_for_message('ObjectRecognition', Point)
        rospy.Subscriber('obj_pos', Point, self.set_cmd_vel)

        rospy.loginfo("Object detected. Starting follower...")
        self.update()

    def set_cmd_vel(self,pos):
        try:
            if not self.target_visible:
                rospy.loginfo("FOLLOWER is Tracking Target!")
            self.target_visible = True
        except:
            # If target is lost, stop the robot by slowing it incrementally
            self.move_cmd.linear.x /= 1.5
            self.move_cmd.angular.z /= 1.5
            
            if self.target_visible:
                rospy.loginfo("FOLLOWER LOST Target!")
            self.target_visible = False
            
            return
        
        # pose with resepect to robot frame
        current_pose = pos
        current_x = current_pose.x 
        current_y = current_pose.y
        current_z = current_pose.z

        # Rotate the robot only if the displacement of the target exceeds the threshold
        if abs(current_x-self.goal_x) > self.x_threshold:
            # Set the rotation speed proportional to the displacement of the target
            speed = (current_x-self.goal_x) * self.x_scale
            self.move_cmd.angular.z = copysign(max(self.min_angular_speed, min(self.max_angular_speed, abs(speed))), speed)
        else:
            self.move_cmd.angular.z = 0.0
 
        # Now get the linear speed
        if abs(current_z - self.goal_z) > self.z_threshold:
            speed = (current_z - self.goal_z) * self.z_scale
            if speed < 0:
                speed *= 1.5
            self.move_cmd.linear.x = copysign(min(self.max_linear_speed, max(self.min_linear_speed, abs(speed))), speed)
        else:
            self.move_cmd.linear.x = 0.0

    def getObjPos(self, pos):
        des_pos = pos.pose.position
    
    def getRobPos(self, pos):
        curr_pos = pos.pose.position

    def update(self):
        while not rospy.is_shutdown():
            # Send the Twist command to the robot
            self.vel_pub.publish(self.move_cmd)
            
            # Sleep for 1/self.rate seconds
            # rospy.sleep()
    
    def shutdown(self):
        rospy.loginfo("Stopping the Visual Servoing...")
        self.vel_pub.publish(Twist())
        rospy.sleep(1) 

if __name__=='__main__':
    try:
        pbvs = PBVS()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Terminating Visual Servo")