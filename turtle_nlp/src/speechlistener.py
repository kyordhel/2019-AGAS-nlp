#!/usr/bin/env python

import os
import signal
import sys
# import time
import rospy
import std_srvs.srv
from std_msgs.msg import String

class SpeechListener:
    def __init__(self, name='asr_listener', callback=None):
        rospy.init_node('robot', anonymous=True)
        rate = rospy.Rate(10)
        if callback is None:
            callback = self._asr_callback_handler
        self.asr_subscriber = rospy.Subscriber('asr', String, callback)
        self._rate = 10
        self.rate = rospy.Rate(self._rate)

    def _asr_callback_handler(self, msg):
        print('Received: {}'.format(msg.data))
    #end def

    def idle(self):
        self.rate.sleep()
    #end def

if __name__ == "__main__":
    node = SpeechListener()
    def signal_handler(sig, frame):
        node.stop()
        print()
        sys.exit(0)
    signal.signal(signal.SIGINT, signal_handler)
    while True:
        node.idle()
