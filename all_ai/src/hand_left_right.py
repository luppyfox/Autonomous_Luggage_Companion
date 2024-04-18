#!/usr/bin/env python3

import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2
import mediapipe as mp

def hand_detection():
    # Set up MediaPipe Hand model
    mp_drawing = mp.solutions.drawing_utils
    mp_drawing_styles = mp.solutions.drawing_styles
    mp_hands = mp.solutions.hands
    hands = mp_hands.Hands()

    rospy.init_node('hand_detection_publisher', anonymous=True)
    image_pub = rospy.Publisher('/camera/color/image_raw', Image, queue_size=10)
    bridge = CvBridge()

    #cap = cv2.VideoCapture(0)  # Use default camera
    
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            continue
        
        # Convert the image to RGB
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        
        # Detect hands in the frame
        results = hands.process(frame_rgb)
        
        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                mp_drawing.draw_landmarks(
                    frame,
                    hand_landmarks,
                    mp_hands.HAND_CONNECTIONS,
                    mp_drawing_styles.get_default_hand_landmarks_style(),
                    mp_drawing_styles.get_default_hand_connections_style()
                )

                #hand idx 0
                xh0 = 0
                yh0 = 0

                for idx, landmark in enumerate(hand_landmarks.landmark):
                    # Convert normalized coordinates to pixel coordinates
                    h, w, _ = frame.shape
                    #print(h)

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
                    
                    # Draw a circle at each hand landmark
                    #cv2.circle(frame, (x, y), 5, (0, 255, 0), -1)

        # Convert the frame to ROS Image message and publish
        image_pub.publish(bridge.cv2_to_imgmsg(frame, "bgr8"))
        
    # Clean up
    cap.release()

if __name__ == '__main__':
    try:
        hand_detection()
    except rospy.ROSInterruptException:
        pass
