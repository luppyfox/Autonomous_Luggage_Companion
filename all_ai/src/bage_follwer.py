#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String, Int32 , Float64
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics  import YOLO
import threading

class obj_pos() :
    def __init__(self, model_path, conf_threshold, FOV_X, FOV_Y) -> None :
        rospy.init_node('bage_follwer', anonymous=True)
        self.conf_threshold = conf_threshold
        self.model = YOLO(model_path)
        self.FOV = [FOV_X, FOV_Y]
        

        rospy.Subscriber("/camera/color/image_raw", Image, self.rgb_callback)
        rospy.Subscriber("/camera/depth/image_raw", Image, self.depth_callback)

        self.pos_arm = rospy.Publisher("/arm_xyz", Float64, queue_size = 10)


        self.bridge = CvBridge()
        self.lock = threading.Lock()

        self.debug_revers = False

        self.rate = rospy.Rate(1000)

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

    def call_back(self):
            while not rospy.is_shutdown():
                if hasattr(self, 'frame_rgb') and hasattr(self, 'frame_depth'):
                    
                        self.lock.acquire()
                      
                        xyz = self.get_obj_pos(self.frame_rgb , self.frame_depth)


            
                        cv2.imshow("RGB", self.frame_rgb)
                        cv2.imshow("Depth", self.frame_depth)
                        self.lock.release()

                        cv2.waitKey(1)
                
                self.rate.sleep()

            


        
    def get_obj_pos(self, image, depth, get_leftmost = False, get_rightmost = False) :
        # image = cv2.resize(image, (320, 320))
        results = self.model.track(image, persist=True, verbose=False)
        
        boxes = results[0].boxes.xyxy.cpu()
        save_cen = [image.shape[1]/2, image.shape[0]/2]
        if results[0].boxes.id is not None:
            for res in boxes  :

                bb = res

                #print(bb)

                leftmost = bb[0]
                rightmost = bb[1]
                topmost = bb[2]
                bottommost = bb[3]



                centroi = [int(rightmost - leftmost), int(bottommost - topmost)]
                
                #print(centroi)
                dist =  self.frame_depth[centroi[1], centroi[0]]

                frame_ = results[0].plot()
                cv2.imshow("bag",frame_)

                if get_leftmost and not get_rightmost :
                  if centroi[0] < save_cen[0] :
                      save_cen = centroi
                elif get_rightmost and not get_leftmost :
                  if centroi[0] > save_cen[0] :
                      save_cen = centroi
                else :
                    save_cen = centroi
                    break
            arm_pos = self.cam_pos_to_cartesian_pos(*save_cen, dist , *self.FOV, *image.shape[:2])
            self.pos_arm.publish(arm_pos)

    def cam_pos_to_cartesian_pos(self, px : int, py : int, dis : float, FOV_X : float, FOV_Y : float, image_width : int, image_height : int) -> list :
        """Converting camera position (Px, Py, Dist.) to cartesian position (X, Y, Z) relative on camera itself.

        Args:
            px (int): image horizontal position. (pixel)
            py (int): image vertical position. (pixel)
            dis (float): distant from camera to object. (any measurement unit)
            FOV_h (float): horizontal FOV. (Must be degree!)
            FOV_v (float): vertical FOV. (Must be degree!)
            image_width (int): width size of the image (pixel)
            image_height (int): height size of the image (pixel)

        Returns:
            list: position of object (X, Y, Z)
        """
        cpx = px - (image_width / 2)
        cpy = -py + (image_height / 2)
        
        theta_x = (FOV_X * cpx) / image_width
        theta_y = (FOV_Y * cpy) / image_height
        x = dis * np.sin(np.rad2deg(theta_x))
        y = x / np.tan(np.rad2deg(theta_x))
        z = dis * np.sin(np.rad2deg(theta_y))
        
        return [x, y, z]
    
    def run(self):
        threading.Thread(target=self.call_back).start()
        rospy.spin()
    
if __name__ == "__main__" :
    obj_fol = obj_pos("catkin_ws/src/ksuck/src/bag_detect_best.pt", 0.1, 60.0, 49.5)
    obj_fol.run()
    
    # cap = cv2.VideoCapture(0)
    # while True :
    #     ret, image = cap.read()
    #     cv2.imshow("awdaw", image)
    #     xyz = obj_fol.get_obj_pos(image)
    #     print(xyz)
        
    #     k = cv2.waitKey(1)
    #     if k == 27 :
    #         break
    # cv2.destroyAllWindows()
