#!/usr/bin/env python

import os
import signal
import sys
# import time
import rospy
import std_srvs.srv
from std_msgs.msg import String
from pocketsphinx import LiveSpeech, get_model_path


class SpeechRecognizerNode:
    _running = None
    _model_path = None
    _data_path = None

    def __init__(self, name='asr'):
        rospy.init_node('robot', anonymous=True)
        rate = rospy.Rate(10)
        self.asr_publisher = rospy.Publisher('asr', String, queue_size=10)
        self._rate = 10
        self.rate = rospy.Rate(self._rate)
        # Model path
        this_path = os.path.dirname(os.path.abspath(__file__))
        self._model_path = os.path.abspath(os.path.join(this_path, "../model/"))
        self._data_path = os.path.abspath(os.path.join(this_path, "../model/data"))

    def run_asr(self):
        print("Initializing live speech recognizer with PocketSphinx...")
        speech = LiveSpeech(
            verbose=False,
            # verbose=True,
            sampling_rate=16000,
            buffer_size=2048,
            no_search=False,
            full_utt=False,
            hmm=os.path.join(self._model_path, 'en-us/en-us'),
            lm=os.path.join(self._model_path, 'en-us/en-us.lm.bin'),
            dic=os.path.join(self._data_path, 'en-us/robot.dict')
        )
        print("Done!")
            # dic=os.path.join(self._model_path, 'cmudict-en-us.dict')

        print("Listening...")
        for phrase in speech:
            reco = str(phrase)
            print('Recognized: {}'.format(reco))
            self.asr_publisher.publish(reco)
    #end def

    def idle(self):
        self.rate.sleep()
    #end def

if __name__ == "__main__":
    node = SpeechRecognizerNode()
    def signal_handler(sig, frame):
        node.stop()
        print()
        sys.exit(0)
    signal.signal(signal.SIGINT, signal_handler)
    node.run_asr()
