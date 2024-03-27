#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import mediapipe as mp
from std_msgs.msg import String
from playsound import playsound
import threading
import time

class HandGestureListener:
    def __init__(self):
        rospy.init_node('hand_gesture_listener', anonymous=True)
        rospy.Subscriber("/camera/color/image_raw", Image, self.callback)
        self.bridge = CvBridge()
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_drawing_styles = mp.solutions.drawing_styles
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands()

        self.han_pub = rospy.Publisher("/hand_send", String, queue_size = 10)

        #flip cam
        self.debug_revers = False

        self.blockvocie = 0
        
        self.rate = rospy.Rate(500)

    def callback(self, data):
        try:
            frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
            if self.debug_revers == True:
                frame = cv2.flip(frame, 1)

            results = self.hands.process(frame)

            if results.multi_hand_landmarks:
                for hand_landmarks in results.multi_hand_landmarks:
                    self.mp_drawing.draw_landmarks(
                        frame,
                        hand_landmarks,
                        self.mp_hands.HAND_CONNECTIONS,
                        self.mp_drawing_styles.get_default_hand_landmarks_style(),
                        self.mp_drawing_styles.get_default_hand_connections_style()
                    )

    
                    #print(hand_landmarks.landmark[0])

                 
                    h, w, _ = frame.shape
                    
                    hand_8x = hand_landmarks.landmark[8].x * w
                    hand_8y = hand_landmarks.landmark[8].y * h

                    hand_0x = hand_landmarks.landmark[0].x * w
                    hand_0y = hand_landmarks.landmark[0].y * h

                      

                        
                    rospy.loginfo(f"Landmark 8: ({hand_landmarks.landmark[8].x * h}, {hand_landmarks.landmark[8].y * w})")
            
                    if (hand_8x > hand_0x) and hand_8y  < hand_0y:
                        rospy.loginfo("left")
                        self.han_pub.publish('left')
                        #self.play_sound('catkin_ws/src/ksuck/src/l.m4a')
                        if self.blockvocie != 1:
                            threading.Thread(target=self.play_sound, args=('catkin_ws/src/ksuck/src/l.m4a',)).start()



                    if (hand_8x  < hand_0x) and hand_8y  < hand_0y:
                        rospy.loginfo("right")
                        self.han_pub.publish('right')
                        #self.play_sound('catkin_ws/src/ksuck/src/r.m4a')
                        if self.blockvocie != 1:
                            threading.Thread(target=self.play_sound, args=('catkin_ws/src/ksuck/src/r.m4a',)).start()
                
                        

            cv2.imshow("Image", frame)
            cv2.waitKey(1)
           
            self.rate.sleep()

        except Exception as e:
            rospy.logerr(e)

    def play_sound(self,file_path):
        self.blockvocie = 1
        playsound(file_path)
        #time.sleep(0.5)
        self.blockvocie = 0

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    hand_gesture_listener = HandGestureListener()
    hand_gesture_listener.run()
