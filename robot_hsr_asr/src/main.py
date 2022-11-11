#!/usr/bin/env python3

# standard libraries
import rospy
from threading import Thread
from queue import Queue
import speech_recognition as sr
from playsound import playsound

# internal classes
# none

# standard messages
from std_msgs.msg import String

# custom messages
# none

# constants and parameters
# none

class Main():
    def __init__(self):
        # set up ROS
        rospy.init_node('robot_hsr_asr')

        self.ros_pub_text = rospy.Publisher('/robot_hsr_asr/text', String, queue_size=10)

        self.r = sr.Recognizer()
        self.audio_queue = Queue()

        self.sound_path = '/home/administrator/catkin_ws/src/ralt_hsr_asr/src/sounds/tone_beep.wav'

        self.run()

    def run(self):
        recognize_thread = Thread(target=self.recognize_worker)
        recognize_thread.daemon = True
        recognize_thread.start()
        with sr.Microphone() as source:
            self.r.adjust_for_ambient_noise(source)
            try:
                while True and not rospy.is_shutdown():
                    print('Listening...')
                    self.audio_queue.put(self.r.listen(source))
            except KeyboardInterrupt:
                pass

        self.audio_queue.join()
        self.audio_queue.put(None)
        recognize_thread.join()

    def recognize_worker(self):
        print('Started worker thread.')
        while True:
            audio = self.audio_queue.get()
            if audio is None: break
            try:
                utterance = self.r.recognize_google(audio)
                print("Utterance:", utterance)
                splits = utterance.split()
                if splits[0] == 'google' or splits[0] == 'Google':
                    msg = String()
                    msg.data = utterance.lstrip('google')
                    self.ros_pub_text.publish(utterance)
            except sr.UnknownValueError:
                print("Could not understand audio.")
            except sr.RequestError as e:
                print("Could not request results from Google Speech Recognition service; {0}".format(e))

            print('Terminated worker thread.')
            self.audio_queue.task_done()

if __name__ == '__main__':
    m = Main()