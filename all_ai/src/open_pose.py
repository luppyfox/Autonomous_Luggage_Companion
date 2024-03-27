#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import threading
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
import numpy as np
from std_msgs.msg import String, Int32 , Float64
import math

class HandGestureListener:
    def __init__(self):
        rospy.init_node('hand_gesture_listener', anonymous=True)
        rospy.Subscriber("/camera/color/image_raw", Image, self.rgb_callback)
        rospy.Subscriber("/camera/depth/image_raw", Image, self.depth_callback)

        self.side_pub = rospy.Publisher("/human_side", String, queue_size = 10)
        self.dist_pub = rospy.Publisher("/human_dist", Int32, queue_size = 10)
        self.turning_pub = rospy.Publisher("/human_turn", Float64, queue_size = 10)

        #pose
        self.mp_pose = mp.solutions.pose
        self.pose = self.mp_pose.Pose(static_image_mode=False, min_detection_confidence=0.5, min_tracking_confidence=0.5)

        #cv
        self.bridge = CvBridge()
        self.lock = threading.Lock()

        # robot maximum rotate speed (deg/s)
        self.maximum_turning_speed = 15.0

        self.debug_revers = False
    
        self.rate = rospy.Rate(500)

    def rgb_callback(self, data):
        try:
            frame_rgb = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.lock.acquire()
            self.frame_rgb = frame_rgb
            if self.debug_revers == True:
                self.frame_rgb = cv2.flip(self.frame_rgb, 1)

            self.lock.release()
        except Exception as e:
            rospy.logerr(e)

    def depth_callback(self, data):
        try:
            frame_depth = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
            self.lock.acquire()
            self.frame_depth = frame_depth
            if self.debug_revers == True:
                self.frame_depth = cv2.flip(self.frame_depth, 1)

            self.lock.release()
        except Exception as e:
            rospy.logerr(e)

    def show_images(self):
        while not rospy.is_shutdown():
            if hasattr(self, 'frame_rgb') and hasattr(self, 'frame_depth'):
                try:
                    self.lock.acquire()
                    results = self.pose.process(self.frame_rgb)
                    if results.pose_landmarks:
                        mp_drawing = mp.solutions.drawing_utils
                        mp_drawing.draw_landmarks(
                        self.frame_rgb, results.pose_landmarks, self.mp_pose.POSE_CONNECTIONS)

                        nose_landmark = results.pose_landmarks.landmark[0]
                        height, width, _ = self.frame_rgb.shape
                        #print(height)
                        cx, cy = int(nose_landmark.x * width), int(nose_landmark.y * height)
                        cv2.circle(self.frame_rgb, (cx, cy), 5, (255, 0, 255), cv2.FILLED)
                        cv2.putText(self.frame_rgb, f"{0}", (cx, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)
                        '''
                        str_data = "midle"
                        str_data = "left" if cx > (width * 2 / 3) else str_data
                        str_data = "midle" if cx <= (width * 2 / 3) and cx >= (width / 3) else str_data
                        str_data = "right" if cx < (width / 3) else str_data
                        '''

                        '''
                        rospy.loginfo(str_data)
                        self.side_pub.publish(str_data)

                        '''
                        #line 
                        #print(cx)
                        cv2.line(self.frame_rgb,(cx, 0),(cx, int(height/2)), (0,255,0), thickness=2)
                        cv2.line(self.frame_rgb,(int(width/2), 0),(int(width/2), int(height/2)), (255,0,0), thickness=2)

                        #center to cx - self.center h = distance 

                        try:
                            pixel_dist = -cx + (width/2)
                            turning = (pixel_dist / width) * self.maximum_turning_speed

                            self.turning_pub.publish(turning)
                        except Exception as e:
                            rospy.logerr(e)
                        try:
                            dist =  self.frame_depth[cy, cx]
                            #print(dist)
                            self.dist_pub.publish(dist)
                        except Exception as e:
                            rospy.logerr(e)                                    
                        

                    cv2.imshow("RGB", self.frame_rgb)
                    cv2.imshow("Depth", self.frame_depth)
                    self.lock.release()
                    cv2.waitKey(1)
                except Exception as e:
                    rospy.logerr(e)
            self.rate.sleep()

    def run(self):
        threading.Thread(target=self.show_images).start()
        rospy.spin()

if __name__ == '__main__':
    hand_gesture_listener = HandGestureListener()
    hand_gesture_listener.run()
