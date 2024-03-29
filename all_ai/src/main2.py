#!/usr/bin/env python3

import rospy
import numpy as np
from std_msgs.msg import String, Int32, Float64
from geometry_msgs.msg import Pose2D, Quaternion, Pose2D
from nav_msgs.msg import Odometry
import math
from geometry_msgs.msg import Twist
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from math import pi
# from pyArmIK import

class MainSystem:
    def __init__(self):
        rospy.init_node('main_node', anonymous=True)
    # ---------------------------- Publish ---------------------------------
        self.state_pub = rospy.Publisher("/state", Int32, queue_size=10)

        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.vel_msg = Twist()
        self.vel_msg.linear.x = 0.0
        self.vel_msg.angular.z = 0.0

    # ---------------------------- Subscribe ---------------------------------
        rospy.Subscriber("/hand_side", String, self.hand_callback_fixed_pose)

        rospy.Subscriber("/bag_pose", String, self.bag_callback)

        # rospy.Subscriber("/human_side", String, queue_size = 10)
        rospy.Subscriber("/human_dist", Int32, self.human_dist_callback)
        self.human_dist = 0
        rospy.Subscriber("/human_turn", Float64, self.human_turn_callback)
        self.human_turn = 0.0

        rospy.Subscriber("/recognized_text", String, self.voice_callback)
        self.voice_text = ""

        rospy.Subscriber('pose2D', Pose2D, self.pose_callback)

    # ---------------------------- Other ---------------------------------

        self.state = 3  #Control manual state
        self.state_pub.publish(self.state)
        rospy.loginfo(self.state)

        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

        self.rate = rospy.Rate(500)

    
    # ---------------------------------------------------------------------    STATE 0  Finding hand side and go to that's bag side-----------------------------------------------------------------------------------------
    def hand_callback_fixed_pose(self, data):
        rospy.loginfo(self.state)
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
                self.state_pub.publish(self.state)
                break
        
        def hand_callback_bag_detect(self, data):
            pass
        
    # ---------------------------------------------------------------------    STATE 1  Finding bag position for manipulator-----------------------------------------------------------------------------------------
    def bag_callback(self, data):
        rospy.loginfo(self.state)
        bag_goal_x = data.x
        bag_goal_y = data.y
        bag_goal_th = data.theta
        # Add code arm control
        self.state = 2
        self.state_pub.publish(self.state)
        pass

    # ---------------------------------------------------------------------    STATE 2 Followiing human-----------------------------------------------------------------------------------------
    def voice_callback(self, data):
        if self.state == 2:
            self.voice_text = data.data

    def human_dist_callback(self,data):
        if self.state == 2:
            self.human_dist = data.data / 1000 # from mm to m

    def human_turn_callback(self, data):
        rospy.loginfo(self.state)
        if self.state == 2:
            self.human_turn = data.data
            vel_x = 0.08*self.human_dist/0.15

            if vel_x >= 0.15:
                vel_x = 0.08  # Maximum speed
            elif self.human_dist <= 0.05:
                vel_x = 0.0
            if self.human_turn > 0.349066:
                self.human_turn = 0.349066
            elif self.human_turn <= 0:
                self.human_turn = 0

            self.vel_msg.linear.x = vel_x
            self.vel_msg.angular.z = self.human_turn
            self.vel_pub.publish(self.vel_msg)

            if self.voice_text == "stop":
                self.vel_msg.linear.x = 0
                self.vel_msg.angular.z = 0
                self.vel_pub.publish(self.vel_msg)
                self.state = 3
                self.state_pub.publish(self.state)

    # ---------------------------------------------------------------------    STATE 3 Navigation-----------------------------------------------------------------------------------------
    def send_goal(self, x, y, theta):
        rospy.loginfo(self.state)
        self.client.wait_for_server()
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        # Set the position
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        
        # Convert the Euler angle to a quaternion.
        q = quaternion_from_euler(0, 0, theta)
        goal.target_pose.pose.orientation = Quaternion(*q)

        # Sends the goal to the action server.
        self.client.send_goal(goal)
        wait = self.client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
        else:
            self.state = 4
            self.state_pub.publish(self.state)
            return self.client.get_result()
    
    # ---------------------------------------------------------------------    STATE 4 Close-----------------------------------------------------------------------------------------

    def pose_callback(self, data):
        self.x = data.x
        self.y = data.y
        self.th = data.theta

    def run(self):
        while not rospy.is_shutdown():
            if self.state == 3:
                rospy.loginfo(self.state)
                self.send_goal(0,0,0)
                rospy.loginfo(self.state)
            rospy.spin()

if __name__ == '__main__':
    hand_gesture_listener = MainSystem()
    hand_gesture_listener.run()