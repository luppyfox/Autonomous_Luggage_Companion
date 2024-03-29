#!/usr/bin/env python


import os
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "hide"
os.environ['SDL_AUDIODRIVER'] = "dummy"

import rospy
import speech_recognition as sr
from std_msgs.msg import String

'''
เพิ่มการเปิดปิดเป็นช่วงได้
'''

class VoiceRecognitionNode:
    def __init__(self):
        rospy.init_node('voice_recognition_node', anonymous=True)
        self.recognizer = sr.Recognizer()

        # Publisher for recognized text
        self.text_pub = rospy.Publisher('/recognized_text', String, queue_size=10)

    def recognize_speech(self):
        with sr.Microphone() as source:
            rospy.loginfo("กรุณาพูดอะไรสักคำ...")
            self.recognizer.adjust_for_ambient_noise(source)
            audio = self.recognizer.listen(source)

            try:
                recognized_text = self.recognizer.recognize_google(audio, language='th')  #en-US
                rospy.loginfo("การรับรู้เสียง: " + recognized_text)
                self.text_pub.publish(recognized_text)
            except sr.UnknownValueError:
                rospy.logwarn("ไม่สามารถรับรู้เสียงได้")
                self.text_pub.publish(recognized_text)
            except sr.RequestError as e:
                rospy.logerr("ไม่สามารถเชื่อมต่อกับ Google Speech Recognition ได้: {0}".format(e))

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            self.recognize_speech()
            rate.sleep()

if __name__ == '__main__':

    try:
        voice_recognition_node = VoiceRecognitionNode()
        voice_recognition_node.run()
    except rospy.ROSInterruptException:
        pass
