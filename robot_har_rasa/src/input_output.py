#!/usr/bin/env python3
import pyttsx3
import speech_recognition as sr
from playsound import playsound
import rospy
import sys
import actionlib
from actionlib_msgs.msg import GoalStatus

from tmc_msgs.msg import Voice, TalkRequestAction, TalkRequestGoal

from log import Log

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

        try:
            if not self.ros_as_talk.wait_for_server(rospy.Duration(20)):
                self.logger.log_warn('Talk action server could not be found.')
        except Exception as e:
            rospy.logerr(e)
            sys.exit(1)

        self.r = sr.Recognizer()
        sr.Microphone.list_microphone_names()

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