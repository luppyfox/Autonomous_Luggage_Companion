#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import mediapipe as mp

class HandGestureListener:
    def __init__(self):
        rospy.init_node('hand_gesture_listener', anonymous=True)
        rospy.Subscriber("/camera/color/image_raw", Image, self.callback)
        self.bridge = CvBridge()
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_drawing_styles = mp.solutions.drawing_styles
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands()

        self.rate = rospy.Rate(500)

    def callback(self, data):
        try:
            frame = self.bridge.imgmsg_to_cv2(data, "bgr8")

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

                    xh0 = 0
                    yh0 = 0

                    for idx, landmark in enumerate(hand_landmarks.landmark):
                        h, w, _ = frame.shape
                        x, y = int(landmark.x * w), int(landmark.y * h)

                        if idx == 0:
                            xh0 = x
                            yh0 = y

                        if idx == 8:
                            rospy.loginfo(f"Landmark {idx}: ({x}, {y})")
                            if (x > xh0) and y < yh0:
                                rospy.loginfo("left")
                            else:
                                rospy.loginfo("right")

            cv2.imshow("Image", frame)
            cv2.waitKey(1)
           
            self.rate.sleep()

        except Exception as e:
            rospy.logerr(e)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    hand_gesture_listener = HandGestureListener()
    hand_gesture_listener.run()
