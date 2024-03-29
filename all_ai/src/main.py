#!/usr/bin/env python3

import rospy
import numpy as np
from std_msgs.msg import String, Int32, Float64
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Odometry
import math
from geometry_msgs.msg import Twist
from pyArmIK import

class MainSystem:
    def __init__(self):
        rospy.init_node('main_node', anonymous=True)

        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.vel_msg = Twist()
        self.vel_msg.linear.x = 0.0
        self.vel_msg.angular.z = 0.0

        rospy.Subscriber("/hand_side", String, self.hand_callback)

        rospy.Subscriber("/bag_pose", String, self.bag_callback)

        # rospy.Subscriber("/human_side", String, queue_size = 10)
        rospy.Subscriber("/human_dist", Int32, self.human_dist_callback)
        rospy.Subscriber("/human_turn", Float64, self.human_turn_callback)

        

        rospy.Subscriber('pose2D', Pose2D, self.pose_callback)

        self.state = 0

        self.rate = rospy.Rate(500)
    
    def hand_callback(self, data):
        tolerance_xy = 0.05 #m
        tolerance_th = 0.02 #radian
        bag_goal_x = 1
        bag_goal_y = 1
        bag_goal_th = 0
        
        if (data.data == "L"):
            dir = 1

        elif (data.data == "R"):
            dir = -1

        else:
            dir = 0

        bag_goal_x *= dir
        bag_goal_y *= dir
        bag_goal_th *= dir
        while (self.state == 0 ):
            dx = bag_goal_x - self.x
            dy = bag_goal_y - self.y
            angle_to_walk_linear = math.atan2(dy, dx)
            dth = angle_to_walk_linear - self.th
            self.vel_msg.angular.z = dth #rad/s
            
            self.vel_pub.publish(self.vel_msg)

            if dth <= tolerance_th and dth >= -tolerance_th:
                self.state = 1
                break
        

    def human_dist_callback(self,data):
        self.human_dist = data.data

    def human_turn_callback(self, data):
        se

    def bag_callback(self, data):
        bag_goal_x = data.x
        bag_goal_y = data.y
        bag_goal_th = data.theta
        pass



    def pose_callback(self, data):
        self.x = data.x
        self.y = data.y
        self.th = data.theta
    
    def depth_callback(self, data):
        try:
            pass
        except:
            pass

    def run(self):
        while not rospy.is_shutdown():
            pass
            rospy.spin()

if __name__ == '__main__':
    hand_gesture_listener = MainSystem()
    hand_gesture_listener.run()