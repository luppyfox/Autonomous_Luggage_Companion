#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib

class GoalSender:
    def __init__(self):
        rospy.init_node('goal_sender_node', anonymous=True)

        # Publisher to publish the goal pose
        self.goal_publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)

        # Action client to monitor the status of the move_base action
        self.action_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.action_client.wait_for_server()

        # Subscribe to the move_base action feedback
        rospy.Subscriber('/move_base/feedback', MoveBaseAction, self.feedback_callback)

    def send_goal_pose(self, goal_pose):
        # Publish the goal pose
        self.goal_publisher.publish(goal_pose)

    def feedback_callback(self, feedback):
        # Check if the status of the move_base action is succeeded
        if feedback.status.status == GoalStatus.SUCCEEDED:
            rospy.loginfo("Goal reached!")

def main():
    try:
        goal_sender = GoalSender()

        # Define the goal pose
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.pose.position.x = 1.0
        goal_pose.pose.position.y = 1.0
        goal_pose.pose.orientation.w = 1.0

        # Send the goal pose
        goal_sender.send_goal_pose(goal_pose)

        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("Goal sender node stopped.")

if __name__ == '__main__':
    main()
