#!/usr/bin/env python3

import rospy
import numpy as np
from std_msgs.msg import String, Int32, Float64
from geometry_msgs.msg import Pose2D, Quaternion, Pose2D, Twist, PoseStamped
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from nav_msgs.msg import Odometry
import math
import actionlib
from tf.transformations import quaternion_from_euler
from math import pi

class MainSystem:
    def __init__(self):
        rospy.init_node('main_node', anonymous=True)

        # Publisher for velocity commands
        self.vel_pub = rospy.Publisher("/cmd_vel_mux/input/teleop", Twist, queue_size=10)
        self.goal_publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)


        # Velocity message
        self.vel_msg = Twist()
        self.vel_msg.linear.x = 0.0
        self.vel_msg.angular.z = 0.0

        # Subscribe to the move_base action feedback
        rospy.Subscriber("/human_dist", Int32, self.human_dist_callback)
        self.human_dist = 0
        rospy.Subscriber("/human_turn", Float64, self.human_turn_callback)
        self.human_turn = 0.0
        rospy.Subscriber('/move_base/feedback', MoveBaseAction, self.feedback_callback)

        # Action client to monitor the status of the move_base action
        self.action_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.action_client.wait_for_server()

        self.state = 0.0

        # Rate for the control loop
        self.rate = rospy.Rate(10)  # Adjust the rate as needed

    def send_goal_pose(self, x, y, w, z):
        # Publish the goal pose
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.orientation.w = w
        goal_pose.pose.orientation.z = z
        self.goal_publisher.publish(goal_pose)

    def feedback_callback(self, feedback):
        # Check if the status of the move_base action is succeeded
        self.goal_reached = feedback.status.status
        if feedback.status.status == GoalStatus.SUCCEEDED:
            rospy.loginfo("Goal reached!")
    
    # Function to send velocity commands
    def send_velocity(self, linear_vel, angular_vel):
        self.vel_msg.linear.x = linear_vel
        self.vel_msg.angular.z = angular_vel
        self.vel_pub.publish(self.vel_msg)

    def human_dist_callback(self,data):
        self.human_dist = data.data / 1000 # from mm to m

    def human_turn_callback(self, data):
        self.human_turn = data.data

    def run(self):
        while not rospy.is_shutdown():
            
            rospy.loginfo("Current state is ", self.state)
            while self.state == 0:
                self.send_goal_pose(-1.55, 2.10, 1.0, 0.0) # go to living room and ready to follow human
                if self.goal_reached == GoalStatus.SUCCEEDED:
                    rospy.loginfo("Goal reached!")
                    self.state = 1
                    break
                self.rate.sleep()

            rospy.loginfo("Current state is ", self.state)
            while self.state == 1:
                if self.human_dist >= 1.5:
                    vel_x = 0.08  # Maximum speed
                elif self.human_dist <= 0.5:
                    vel_x = 0.0
                if self.human_turn > 0.349066:
                    ang_z = 0.349066
                elif self.human_turn <= 0:
                    ang_z = 0.0
                self.send_velocity(vel_x, ang_z)  # Adjust linear velocity as needed
                if vel_x == 0.0 and ang_z == 0.0:
                    self.send_velocity(0, 0)
                    self.state = 2
                    break
                self.rate.sleep()
            
            rospy.loginfo("Current state is ", self.state)
            while self.state == 2:
                pass

            self.rate.sleep()

if __name__ == '__main__':
    hand_gesture_listener = MainSystem()
    hand_gesture_listener.run()