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

        self.logger.log('Loading Whisper model...')
        self.r = sr.Recognizer()
        self.r.dynamic_energy_threshold = False
        self.r.energy_threshold = 2000
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
                    self.logger.log('Listening...')
                    timed_out = False
                    try:
                        audio = self.r.listen(source, timeout=1)
                    except sr.WaitTimeoutError:
                        timed_out = True
                    if not timed_out:
                        self.audio_queue.put(audio)
            except KeyboardInterrupt:
                pass

        self.audio_queue.join()
        self.audio_queue.put(None)
        recognize_thread.join()

    def recognize_worker(self):
        self.logger.log('Started worker thread.')
        while True:
            audio = self.audio_queue.get()
            if audio is None:
                break

            utterance = self.r.recognize_whisper(audio, language='English', model='base')
            log = 'Utterance:' + utterance
            self.logger.log(log)
            words = utterance.split()
            if len(words) > 0:
                hotword = words[0]
                hotword = hotword.lower()
                hotword = hotword[0:5]
                if hotword == 'david':
                    utterance = utterance.lstrip('google')
                    msg = String()
                    msg.data = utterance
                    self.ros_pub_rasa_utterance_internal.publish(msg)

            self.logger.log('Terminated worker thread.')
            self.audio_queue.task_done()
            
    def listen_once(self):
        with sr.Microphone() as source:
            self.logger.log('Adjusting for ambient noise...')

            path = self.rel_path + '/src/sounds/tone_beep.wav'
            playsound(path)

            self.logger.log('Listening...')
            try:
                audio = self.r.listen(source, timeout=1)
            except sr.WaitTimeoutError:
                self.logger.log_warn('No audio detected.')
        try:
            # utterance = self.r.recognize_google_cloud(audio, language='en-GB')
            utterance = self.r.recognize_whisper(audio, language='English', model='base')
            log = 'Utterance:' + utterance
            self.logger.log(log)
            return utterance
        except sr.UnknownValueError:
            self.logger.log_warn('Could not understand audio.')

if __name__ == '__main__':
    io = InputOutput()
    io.listen()