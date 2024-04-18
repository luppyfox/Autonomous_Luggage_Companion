#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String, Int32 , Float64

import cv2
import numpy as np
import threading

import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision

from ultralytics import YOLO

class hand_pose:
    def __init__(self, cam_color, cam_depth, model_yolo = '', revers_cam = False, ros_rate = 500):
        #ชื่อ node
        rospy.init_node('hand_gesture_listener', anonymous=True)
        rospy.Subscriber(cam_color, Image, self.rgb_callback)
        rospy.Subscriber(cam_depth, Image, self.depth_callback)

        self.bridge = CvBridge()
        self.lock = threading.Lock()

        #กลับด้านกล้อง
        self.debug_revers = revers_cam

        #ความเร็วการทำงานกล้อง
        self.rate = rospy.Rate(ros_rate)

        self.model = YOLO(model_yolo)
        


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



    def main_loop(self):
        while not rospy.is_shutdown():
            if hasattr(self, 'frame_rgb') and hasattr(self, 'frame_depth'):
                try:
                    self.lock.acquire()
                    results = self.model.track(self.frame_rgb)#จำกัดจำนวน [0]
    
                    #result_keypoint = results.keypoints.xyn.cpu().numpy()


                    result_keypoint = results[0].keypoints.xyn.cpu().numpy()

                    print(result_keypoint)
                    
                    frame_ = results[0].plot()


                    cv2.imshow("Image",frame_)
                    cv2.imshow("RGB", self.frame_rgb)
                    
                    self.lock.release()
                    cv2.waitKey(1)
                except Exception as e:
                    rospy.logerr(e)

                self.rate.sleep()

    def run(self):
        threading.Thread(target=self.main_loop).start()
        rospy.spin()


if __name__ == '__main__':


    #ที่อยู่ topic กล้อง
    topic_camera_color = '/camera/color/image_raw'
    topic_camera_depth = '/camera/depth/image_raw'

    #ที่อยู่ url Ai 
    yolov8_pose = 'buildnaja/src/Autonomous_Luggage_Companion/all_ai/src/yolov8n-pose.pt'
    
    build_naja = hand_pose(topic_camera_color, topic_camera_depth, yolov8_pose)
    build_naja.run()
    