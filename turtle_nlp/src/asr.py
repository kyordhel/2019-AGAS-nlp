#!/usr/bin/env python
#
# Requires: 
#	# apt-get install python-dev python3-dev libasound-dev portaudio19-dev libportaudio2 libportaudiocpp0 ffmpeg libav-tools
#	# pip install SpeechRecognition --upgrade
#	# pip install pyaudio --upgrade
#
# Python 2.7 users might need to run
#	# python2.7 -m pip install ...
#
#
import os
import re
import threading
import speech_recognition as sr

# print sr.__version__

# reco.recognize_google() # Online
# reco.recognize_wit() # Online
# reco.recognize_sphinx()# Offline

# To choose the default microphone
# for index, name in enumerate(sr.Microphone.list_microphone_names()):
# 	print("Microphone with name \"{1}\" found for `Microphone(device_index={0})`".format(index, name))

class Recognizer(threading.Thread):

	def __init__(self):
		threading.Thread.__init__(self)
		self.reco = sr.Recognizer()
		self.running = False
		self.event = threading.Event()
		self.rlock = threading.Lock()
		self.lastRecognition = None
	#end def

	def __del__(self):
		self.running = False
	#end def

	def start(self):
		if self.running:
			return
		self.running = True
		threading.Thread.start(self)
	#end def

	def stop(self):
		if not self.running:
			return
		self.running = False
		self.rlock.release()
		self.event.clear()
	#end def

	def run(self):
		if not self.running:
			return
		try:
			with sr.Microphone() as source:
				self.reco.adjust_for_ambient_noise(source)
				while self.running:
					try:
						audio = self.reco.listen(source)
						temp = self.reco.recognize_google(audio, language='en-US')
						# temp = self.reco.recognize_sphinx(audio, language='en-US')
						self.rlock.acquire()
						self.lastRecognition = temp
						self.rlock.release()
						self.event.set()
					# Could not understand audio
					except LookupError as ex:
						self.lastRecognition = None
						self.event.set()
						continue
					# Other exceptions
					except Exception as ex:
						# print 'Oops While! {}'.format(ex)
						# self.running = False
						self.lastRecognition = None
						self.event.set()
						# return
						continue
				#end while
				self.event.set()
			#end whth
		except Exception as ex:
			print 'Oops! {}'.format(ex)
			self.running = False
			self.event.set()
			return
	#end def

	def recognize(self):
		if not self.running:
			raise Exception('Thread not running, call start first')
		self.event.wait()
		self.event.clear()
		self.rlock.acquire()
		value = self.lastRecognition
		self.rlock.release()
		return value
	#end def

#end class

if __name__ == '__main__':
	reco = Recognizer()
	print 'Initialized (Version {})'.format(sr.__version__)
	reco.start()
	print 'Started'
	reco.recognize()
	while True:
		try:
			print 'Recognized: {}'.format(reco.recognize())
		except:
			reco.stop()
			print 'Failed'

