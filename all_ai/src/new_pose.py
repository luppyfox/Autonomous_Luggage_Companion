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
                    results = self.model.track(self.frame_rgb , persist=False, verbose=False)#จำกัดจำนวน [0]
                    h, w, _ = self.frame_rgb.shape
                    frame_copy = self.frame_rgb.copy()
                    #id
                    track_ids = results[0].boxes.id.int().cpu().tolist()
                    #boxs
                    boxes = results[0].boxes.xyxy.cpu()
                    #keypoint ตำแหน่งกระดูก
                    result_keypoint = results[0].keypoints.xyn.cpu().numpy()

                    #หาตรงกลางตำแหน่งกระดูกช่วงอก
                   
                   
                    btw_distance = []
                    btw_id = 0
                    for res_key , id  in zip(result_keypoint,track_ids):
                        px1 = res_key[5][0] * w
                        py1 = res_key[5][1] * h

                        px2 = res_key[6][0] * w
                        py2 = res_key[6][1] * h

                        if px1 != '' or px2 != '':
                            ctx_p = (px1 + px2) /2
                            cty_p = (py1 + py2) /2

                            cv2.putText(frame_copy, f"{id}", (abs(int(ctx_p)), abs(int(cty_p))), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)
                            cv2.circle(frame_copy , (abs(int(ctx_p)), abs(int(cty_p))), 5, (255, 0, 255), cv2.FILLED)
                            
                            dist =  self.frame_depth[int(cty_p) , int(ctx_p)] #mm


                            #print(dist)
                            if dist < 2000:
                                btw_distance.append((id,dist))
                    
                    min_value = min(btw_distance)
                    x1 , y1 , x2 , y2 = boxes[min_value[0]]
                    #cropped_image = self.frame_rgb[int(y1),int(y2):int(x1),int(x2)]
                    #cv2.rectangle(frame_copy, (int(x1), int(y1)), (int(x2), int(y2)), (255, 0, 255), 2)



                    frame_ = results[0].plot()
                    
                    #cv2.imshow("crop",cropped_image)
                    cv2.imshow("Image",frame_)
                    cv2.imshow("RGB", self.frame_rgb)
                    cv2.imshow("RGB_copy", frame_copy)
                    
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
    