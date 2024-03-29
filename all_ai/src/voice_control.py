#!/usr/bin/env python3

import rospy
import speech_recognition as sr
from std_msgs.msg import String

class VoiceRecognitionNode:
    def __init__(self):
        rospy.init_node('voice_recognition_node', anonymous=True)
        self.recognizer = sr.Recognizer()
        self.rate = rospy.Rate(100)  # เปลี่ยนเป็น 10 Hz
        # สร้าง Publisher สำหรับข้อความที่รับรู้
        self.text_pub = rospy.Publisher('/recognized_text', String, queue_size=10)

    def recognize_speech(self):
        with sr.Microphone() as source:
            rospy.loginfo("กรุณาพูดอะไรสักคำ...")
            self.text_pub.publish("wait for voice !!")
            self.recognizer.adjust_for_ambient_noise(source)
            audio = self.recognizer.listen(source)
            rospy.loginfo("รอแปป...")
            self.text_pub.publish("wait process")
            try:
                recognized_text = self.recognizer.recognize_google(audio, language='en')  # แก้ไขเป็น 'th-TH'
                rospy.loginfo("การรับรู้เสียง: " + recognized_text)
                self.text_pub.publish(recognized_text)
            except sr.UnknownValueError:
                rospy.logwarn("ไม่สามารถรับรู้เสียงได้")
            except sr.RequestError as e:
                rospy.logerr("ไม่สามารถเชื่อมต่อกับ Google Speech Recognition ได้: {0}".format(e))

    def run(self):
        while not rospy.is_shutdown():
            self.recognize_speech()
            self.rate.sleep()

if __name__ == '__main__':
    voice_recognition_node = VoiceRecognitionNode()
    voice_recognition_node.run()
