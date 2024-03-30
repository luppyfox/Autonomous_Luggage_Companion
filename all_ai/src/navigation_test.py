#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

class MainSystem:
    def __init__(self):
        rospy.init_node('main_node', anonymous=True)

        # Publisher for velocity commands
        self.vel_pub = rospy.Publisher("/cmd_vel_mux/input/teleop", Twist, queue_size=10)

        # Velocity message
        self.vel_msg = Twist()
        self.vel_msg.linear.x = 0.0
        self.vel_msg.angular.z = 0.0

        # Rate for the control loop
        self.rate = rospy.Rate(10)  # Adjust the rate as needed

    # Function to send velocity commands
    def send_velocity(self, linear_vel, angular_vel):
        self.vel_msg.linear.x = linear_vel
        self.vel_msg.angular.z = angular_vel
        self.vel_pub.publish(self.vel_msg)

    def run(self):
        while not rospy.is_shutdown():
            # Here you can control the robot's velocity based on user input or any other logic
            # For example, to move forward:
            self.send_velocity(0.1, 0.0)  # Adjust linear velocity as needed
            self.rate.sleep()

if __name__ == '__main__':
    hand_gesture_listener = MainSystem()
    hand_gesture_listener.run()