#!/usr/bin/env python3

# standard libraries
import pyttsx3
import speech_recognition as sr
from playsound import playsound
import rospy
import sys
import actionlib
from queue import Queue
from threading import Thread

# internal classes
from log import Log

# standard messages
from actionlib_msgs.msg import GoalStatus
from std_msgs.msg import String
from tmc_msgs.msg import Voice, TalkRequestAction, TalkRequestGoal

# custom messages
# none

# constants and parameters
# none

class InputOutput(object):
    def __init__(self, rel_path, output):
        self.id = 'input_outpt'
        self.logger = Log(self.id)

        self.rel_path = rel_path

        self.output = output

        if self.output == 'ROBOT':
            self.tts_pub = rospy.Publisher('/talk_request', Voice, queue_size=10)
        elif self.output == 'LOCAL':
            self.tts_engine = pyttsx3.init()
            self.tts_engine.setProperty("rate", 110)
        else:
            self.logger.log_warn('Invalid output mode specified.')
            exit(0)

        talk_as = '/talk_request_action'
        self.ros_as_talk = actionlib.SimpleActionClient(talk_as, TalkRequestAction)
        self.ros_pub_rasa_utterance_internal =  rospy.Publisher('/robot_har_rasa/rasa_utterance_internal', String, queue_size=10)

        try:
            if not self.ros_as_talk.wait_for_server(rospy.Duration(20)):
                self.logger.log_warn('Talk action server could not be found.')
        except Exception as e:
            rospy.logerr(e)
            sys.exit(1)

        self.r = sr.Recognizer()
        sr.Microphone.list_microphone_names()
        self.audio_queue = Queue()

        self.logger.log_great('Ready.')

    def request(self, text):
        if self.output == 'ROBOT':
            log = 'Sending to HSR TTS: ' + text
            self.logger.log(log)

            goal = TalkRequestGoal()
            goal.data.language = Voice.kEnglish
            goal.data.sentence = text

            self.ros_as_talk.send_goal(goal)

            self.ros_as_talk.wait_for_result(timeout=rospy.Duration(40))

        elif self.output == 'LOCAL':
            self.tts_engine.say(text)
            self.tts_engine.runAndWait()

    def listen(self):
        listen_thread = Thread(target=self.listen_worker)
        listen_thread.daemon = True
        listen_thread.start()

    def listen_worker(self):
        recognize_thread = Thread(target=self.recognize_worker)
        recognize_thread.daemon = True
        recognize_thread.start()
        with sr.Microphone() as source:
            try:
                while True and not rospy.is_shutdown():
                    self.r.adjust_for_ambient_noise(source)
                    self.logger.log('Listening...')
                    self.audio_queue.put(self.r.listen(source, phrase_time_limit=5))
            except KeyboardInterrupt:
                pass

        self.audio_queue.join()
        self.audio_queue.put(None)
        recognize_thread.join()

    def recognize_worker(self):
        print('Started worker thread.')
        while True:
            audio = self.audio_queue.get()
            if audio is None:
                break
            try:
                utterance = self.r.recognize_google(audio)
                print("Utterance:", utterance)
                splits = utterance.split()
                if splits[0] == 'google' or splits[0] == 'Google':
                    utterance = utterance.lstrip('google')
                    msg = String()
                    msg.data = utterance
                    self.ros_pub_rasa_utterance_internal.publish(msg)
            except sr.UnknownValueError:
                print("Could not understand audio.")
            except sr.RequestError as e:
                print("Could not request results from Google Speech Recognition service; {0}".format(e))

            print('Terminated worker thread.')
            self.audio_queue.task_done()
            
    def listen_once(self):
        with sr.Microphone() as source:
            log = 'Adjusting for ambient noise...'
            self.logger.log(log)
            self.r.adjust_for_ambient_noise(source)
            path = self.rel_path + '/src/sounds/tone_beep.wav'
            playsound(path)
            log = 'Say something...'
            self.logger.log(log)
            try:
                audio = self.r.listen(source, timeout=10, phrase_time_limit=5)
                log = 'Done listening.'
                self.logger.log(log)
            except sr.WaitTimeoutError:
                msg = 'No audio detected.'
                self.logger.log_warn(msg)
        try:
            log = 'Sending audio to Google Cloud ASR for processing...'
            self.logger.log(log)
            result = self.r.recognize_google_cloud(audio)
            log = 'Google Cloud ASR thinks you said: ' + result
            self.logger.log(log)
            return result
        except sr.UnknownValueError:
            log = 'Google Cloud ASR could not understand audio.'
            self.logger.log_warn(log)
        except sr.RequestError as e:
            print("Could not request results from Google Cloud ASR service: {0}".format(e))

if __name__ == '__main__':
    io = InputOutput()
    io.listen()