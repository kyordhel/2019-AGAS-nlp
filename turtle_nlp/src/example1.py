#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os
import sys
import time
import math
import signal


import re
from robot import Robot
from speechlistener import SpeechListener

robot = Robot()

def parse(s):
	keywords = ['advance', 'step', 'turn', 'stop', 'spin']
	parts = re.split(r'\s+', s.lower())
	for word in parts:
		if word in keywords:
			return 'robot.{}()'.format(word)
	return None
#end def

def execute(command):
	if command in ['exit', 'quit', 'end']:
		robot.stop()
		sys.exit(1)
	action = parse(command)
	if not action is None:
		eval(action)

def speech_handler(message):
	print('I heard: {}'.format(message.data))
	execute(message.data)
#end def

def main():
	listener = SpeechListener('example1', speech_handler)
	while(True):
		command = raw_input('? ')
		execute(command)
#end def





















def exit_gracefully(signum, frame):
	# restore the original signal handler as otherwise evil things will happen
	# in raw_input when CTRL+C is pressed, and our signal handler is not re-entrant
	signal.signal(signal.SIGINT, original_sigint)
	try:
		sys.exit(1)
	except KeyboardInterrupt:
		sys.exit(1)
	# restore the exit gracefully handler here
	signal.signal(signal.SIGINT, exit_gracefully)


if __name__ == '__main__':
	original_sigint = signal.getsignal(signal.SIGINT)
	signal.signal(signal.SIGINT, exit_gracefully)
	main()
