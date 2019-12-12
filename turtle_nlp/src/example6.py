#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os
import sys
import time
import math
import signal


import nltk
from robot import Robot
from asr import Recognizer
robot = Robot()


grammar='''
% start S
S[F=?f, A=''] -> VB_STOP[F=?f]
S[F=?f, A=?a] -> VB_TURN[F=?f] DIR[A=?a]
S[F=?f, A=?a] -> VB_TURN[F=?f] PREP DET DIR[A=?a]
VB_TURN[F='rotate'] -> 'rotate' | 'turn' | 'go' | 'drive' | 'head'
VB_STOP[F='stop'] -> 'stop' | 'halt' | 'break' | ('that\'s' 'enough') | 'enough'
PREP -> 'to' | 'towards'
DET -> 'the'
DIR[A='90'] -> 'left'
DIR[A='-90'] -> 'right'
DIR[A='180'] -> 'back' | 'around'
'''

grammar='''
% start S
S[F=?f, A=?a] -> SS[F=?f,A=?a] | PS[F=?f,A=?a]
PS[F=?f, A=?a] -> LPOL SS[F=?f,A=?a]
PS[F=?f, A=?a] -> SS[F=?f,A=?a] RPOL
PS[F=?f, A=?a] -> LPOL SS[F=?f,A=?a] RPOL


SS[F=?f, A=?a] -> VB_TURN[F=?f] DIR[A=?a]
SS[F=?f, A=?a] -> VB_TURN[F=?f] PREP DET DIR[A=?a]
SS[F=?f, A=''] -> VB_SPIN[F=?f]
SS[F=?f, A=?a] -> VB_SPIN[F=?f] SPDR[A=?a]
SS[F=?f, A=''] -> VB_MOVE[F=?f] 
SS[F=?f, A=?a] -> VB_MOVE[F=?f] SPDL[A=?a]
SS[F=?f, A=''] -> VB_STOP[F=?f]

PREP -> 'to' | 'towards'
DET -> 'the'
DIR[A='90'] -> 'left'
DIR[A='-90'] -> 'right'
DIR[A='180'] -> 'back' | 'around'
SPDR[A='20'] -> 'fast' | 'faster'
SPDR[A='5'] -> 'slow' | 'slower'
SPDL[A='3'] -> 'fast' | 'faster'
SPDL[A='0.5'] -> 'slow' | 'slower'

VB_TURN[F='rotate'] -> 'rotate' | 'turn' | 'go' | 'drive' | 'head'
VB_SPIN[F='spin'] -> 'spin'
VB_MOVE[F='advance'] -> 'walk' | 'run' | 'move' | 'drive' | 'navigate'
VB_STOP[F='stop'] -> 'stop' | 'halt' | 'break' | 'enough'

SS[F=?f, A=?a] -> VB_STRT VB_TURN[F=?f, +ING] DIR[A=?a]
SS[F=?f, A=?a] -> VB_STRT VB_TURN[F=?f, +ING] PREP DET DIR[A=?a]
SS[F=?f, A=''] -> VB_STRT VB_SPIN[F=?f, +ING]
SS[F=?f, A=?a] -> VB_STRT VB_SPIN[F=?f, +ING] SPDR[A=?a]
SS[F=?f, A=''] -> VB_STRT VB_MOVE[F=?f, +ING]
SS[F=?f, A=?a] -> VB_STRT VB_MOVE[F=?f, +ING] SPDL[A=?a]
VB_STRT -> 'start' | 'begin' | 'commence'
VB_TURN[F='rotate', +ING] -> 'rotating' | 'turning' | 'going' | 'driving' | 'head' | 'moving'
VB_SPIN[F='spin', +ING] -> 'spinning' | 'turning'
VB_MOVE[F='advance', +ING] -> 'walking' | 'running' | 'moving' | 'driving' | 'navigating'

LPOL -> PLS | 'ok' | 'okay'
LPOL -> VB_AUX YOU
LPOL -> VB_AUX YOU PLS
RPOL -> PLS
VB_AUX -> 'could' | 'can' | 'would'
PLS -> 'please'
YOU -> 'you'

'''

# print grammar
def parse(s):
	try:
		fgrammar = nltk.grammar.FeatureGrammar.fromstring(grammar)
		fparser = nltk.parse.FeatureChartParser(fgrammar)
		# trees = list(fparser.parse( s.lower().split(' ') ))
		trees = list(fparser.parse(nltk.word_tokenize(s.lower())))
		if len(trees) < 1:
			print("Ungrammatical sentence.")
			return None
		tree = trees[0]
		return 'robot.{}({})'.format( trees[0].label()['F'], trees[0].label()['A'])
	except Exception as ex:
		print ex
		# print("Ex: Ungrammatical sentence.")
		return None
#end def	

def main():
	reco = Recognizer()
	reco.start()
	while(True):
		s = reco.recognize()
		if s in ['exit', 'quit', 'end']:
			robot.stop()
			sys.exit(1)
		action = parse(s)
		if not action is None:
			eval(action)
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
