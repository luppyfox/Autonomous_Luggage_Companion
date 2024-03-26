#!/usr/bin/env python3

import rospy
import numpy as np
from std_msgs.msg import String, Int32

class MainSystem:
    def __init__(self):
        rospy.init_node('main_node', anonymous=True)
        rospy.Subscriber("/camera/color/image_raw", Image, self.rgb_callback)
        rospy.Subscriber("/camera/depth/image_raw", Image, self.depth_callback)

        self.side_pub = rospy.Publisher("/human_side", String, queue_size = 10)
        self.dist_pub = rospy.Publisher("/human_dist", Int32, queue_size = 10)

        #pose
        self.mp_pose = mp.solutions.pose
        self.pose = self.mp_pose.Pose(static_image_mode=False, min_detection_confidence=0.5, min_tracking_confidence=0.5)

        #cv
        self.bridge = CvBridge()
        self.lock = threading.Lock()

        self.rate = rospy.Rate(500)

    def rgb_callback(self, data):
        try:
            frame_rgb = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.lock.acquire()
            self.frame_rgb = frame_rgb
            self.lock.release()
        except Exception as e:
            rospy.logerr(e)

    def depth_callback(self, data):
        try:

    def run(self):
        threading.Thread(target=self.show_images).start()
        rospy.spin()

if __name__ == '__main__':
    hand_gesture_listener = MainSystem()
    hand_gesture_listener.run()