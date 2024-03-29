#!/usr/bin/env python3


import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
from ultralytics import YOLO
import onnxruntime
import numpy as np
import cv2
import pandas
class mainX:
    #global

    def __init__(self) :
        self.model = YOLO('catkin_ws/src/ksuck/src/best.pt')
        self.conf_threshold = 0.4
        rospy.init_node('v8_bag', anonymous=True)
        rospy.Subscriber("/camera/color/image_raw", Image, self.capstart)

        self.bridge = CvBridge()

        self.rate = rospy.Rate(1000)
 
        
    
    #main2
    def capstart(self , data):
        try:
            frame_rgb = self.bridge.imgmsg_to_cv2(data, "bgr8")

            h, w, _ = frame_rgb.shape
            
            
            #frame_rgb_resized = cv2.resize(frame_rgb, (320, 320))

            results = self.model.track(frame_rgb, persist=True, verbose=False)


            
            boxes = results[0].boxes.xyxy
            
            if results[0].boxes.id is not None:
                numberclass = results[0].boxes.cls.tolist()
                numberclass = numberclass[0]

                track_ids = results[0].boxes.id.int().tolist()

                confs = results[0].boxes.conf.float().tolist()
                confs = confs[0]

                clss = results[0].boxes.cls.cpu().tolist()

                boxes = results[0].boxes.xyxy.cpu()

                print(f'clss = {clss} confs = {confs} box = {boxes}')



            frame_ = results[0].plot()
            
            cv2.imshow("Image",frame_)
            cv2.waitKey(1)
            self.rate.sleep()

        except Exception as e:
            rospy.logerr(e)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    v8 = mainX()
    v8.run()
