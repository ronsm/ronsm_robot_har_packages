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
        r = sr.Recognizer()
        r.energy_treshold = 375
        with sr.Microphone() as source:
            print('Say something...')
            audio = r.listen(source, timeout=10, phrase_time_limit=5)
            print('Done listening.')
        try:
            result = r.recognize_google_cloud(audio)
            print("Google Cloud Speech thinks you said", result)
            return result
        except sr.UnknownValueError:
            print("Google Cloud Speech could not understand audio")
        except sr.RequestError as e:
            print("Could not request results from Google Cloud Speech service; {0}".format(e))

if __name__ == '__main__':
    io = InputOutput()
    io.listen()