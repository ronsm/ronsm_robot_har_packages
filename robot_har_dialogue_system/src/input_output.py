#!/usr/bin/env python3
import pyttsx3
import speech_recognition as sr
import rospy

from tmc_msgs.msg import Voice

from log import Log

class InputOutput(object):
    def __init__(self):
        self.id = 'input_outpt'
        self.logger = Log(self.id)

        # self.tts_engine = pyttsx3.init()
        # self.tts_engine.setProperty("rate", 140)

        self.tts_pub = rospy.Publisher('/talk_request', Voice, queue_size=10)

        self.r = sr.Recognizer()
        # self.r.energy_treshold = 375
        sr.Microphone.list_microphone_names()

        self.logger.log_great('Ready.')

    def say(self, text):
        # self.tts_engine.say(text)
        # self.tts_engine.runAndWait()

        log = 'Sending to HSR TTS: ' + text
        self.logger.log(log)

        msg = Voice()
        msg.language = 1
        msg.interrupting = True
        msg.queueing = True
        msg.sentence = text

        self.tts_pub.publish(msg)

        rospy.sleep(5)

    def listen(self):
        with sr.Microphone() as source:
            log = 'Adjusting for ambient noise...'
            self.logger.log(log)
            self.r.adjust_for_ambient_noise(source)
            log = 'Say something...'
            self.logger.log(log)
            try:
                audio = self.r.listen(source, timeout=10, phrase_time_limit=5)
                # os.system('play -n synth 0.25 sin 700')
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