#!/usr/bin/env python3

import rospy
import ub_chat
from ub_common import *

from geometry_msgs.msg import Twist

from ub_web.msg import touchpad, dpad, chat, audio_command, audio_response, audio_transcribed, joystick


'''
To Do:
HTML
	-[ ] Disable "push to talk" button if mic is already on
	-[ ] Designate a joystick button to control mic.  
	-[ ] Add camera controls (e.g., Aruco options, resolution, fps, etc.)

Python
	-[ ] Add olab_camera?		
'''


# ----- See the `interpret()` function below --------------
import re

_regexOptionalMeters = '(?:\d*[.]?\d*\s*)?(?:meter)?'
_regexRequiredMeters = r'\d+[.]?\d*\s*\bmeter'

regexXMeters = '\d*[.]?\d*\s*meter'

regexMove        = r'((\bmove\b)|(\bgo\b))'
regexMoveForward = r'{{}}\s*((forward)|(ahead))\s*(({})|(\Z))'.format(_regexRequiredMeters)
regexMoveBack    = r'{{}}\s*((\bbackwards\b)|(\bbackward\b)|(\bback\b))\s*(({})|(\Z))'.format(_regexRequiredMeters)
regexMoveLeft    = r'{{}}\s*\bleft\b\s*(({})|(\Z))'.format(_regexRequiredMeters)
regexMoveRight   = r'{{}}\s*\bright\b\s*(({})|(\Z))'.format(_regexRequiredMeters)
regexMoveUp      = r'{{}}\s*\bup\b\s*(({})|(\Z))'.format(_regexRequiredMeters)
regexMoveDown    = r'{{}}\s*\bdown\b\s*(({})|(\Z))'.format(_regexRequiredMeters)


def processMatch(regex, text):
	# Match happens only at beginning of text
	mtch = re.match(regex, text)
	if (mtch):
		return (True, mtch.string[mtch.start():mtch.end()].strip())
	else:
		return (False, '')
	
def processSearch(regex, text):
	# Search can be anywhere in text
	srch = re.search(regex, text)
	if (srch):
		return (True, srch.string[srch.start():srch.end()].strip())
	else:
		return (False, '')    

def getDiscreteMovement(txt, regex, moveDict, descr, axis, direction, default, scale):
	'''
	txt -- String of spoken command, from whisper and cleaned
	regex -- regexMoveForward, regexMoveBack, regexMoveLeft, ...
	moveDict -- Dictionary with keys of 'x', 'y', and 'z'
	descr -- A text description of the move, like 'forward 1 meter'
	axis -- 'x', 'y', or 'z'
	direction -- 'forward', 'left', 'up', etc.
	default -- +1 or -1 meter
	scale -- +1 (for forward, right, or down), or -1 (for back, left, or up)

	returns updated moveDict
	'''
	
	(isMatch, substring) = processSearch(regex, txt)
	if (isMatch):
		moveDict[axis] = default
		# Was a distance given?
		(isMatch, subsubstring) = processSearch(_regexRequiredMeters, substring)
		if (isMatch):
			(isMatch, subsubsubstring) = processSearch('\d+[.]?\d*', subsubstring)
			if (isMatch):
				moveDict[axis] = scale * float(subsubsubstring)

		if (abs(moveDict[axis]) == 1):
			descr += ' {} {} meter'.format(direction, abs(moveDict[axis]))
		else:
			descr += ' {} {} meters'.format(direction, abs(moveDict[axis]))

	return (moveDict, descr)
	
# Move/Go
def isMove(txt, robotID=-1, userID=-1):
	'''
	Forward / ahead [1 meter]	+x
	Back / Backwards [1 meter]	-x
	Left [1 meter]				-y
	Right [1 meter]				+y
	up [1 meter]				-z
	down [1 meter]				+z
	'''
	
	(isMatch, matchword) = processSearch(regexMove, txt)
	if (isMatch):

		moveDict = {'x': 0, 'y': 0, 'z': 0}
		descr    = ''
		(moveDict, descr) = getDiscreteMovement(txt, regexMoveForward.format(matchword), moveDict, descr, 'x', 'forward', +1, +1)
		(moveDict, descr) = getDiscreteMovement(txt, regexMoveBack.format(matchword),    moveDict, descr, 'x', 'back',    -1, -1)
		(moveDict, descr) = getDiscreteMovement(txt, regexMoveRight.format(matchword),   moveDict, descr, 'y', 'right',   -1, +1)
		(moveDict, descr) = getDiscreteMovement(txt, regexMoveLeft.format(matchword),    moveDict, descr, 'y', 'left',    +1, -1)
		(moveDict, descr) = getDiscreteMovement(txt, regexMoveDown.format(matchword),    moveDict, descr, 'z', 'down',    -1, +1)
		(moveDict, descr) = getDiscreteMovement(txt, regexMoveUp.format(matchword),      moveDict, descr, 'z', 'up',      +1, -1)
		
		if ((moveDict['x'] != 0) or (moveDict['y'] != 0) or (moveDict['z'] != 0)):
			
			pubAudioResponse(userID, robotID, f'Moving {descr}', f'moving {descr}', SEVERITY_INFO)

			return True, moveDict
			
		pubAudioResponse(userID, robotID, f'Unrecognized move command: {txt}.', f'Unrecognized move command: {txt}.', SEVERITY_ERROR)

		return False, {}	    
	
	else:
		return False, {}

pub_audio_response = rospy.Publisher('audio_response', audio_response, queue_size=1)
def pubAudioResponse(userID, robotID, displayMsg, speakMsg, severity):

	'''
	Publish a message for a user to the `audio_response` topic.
	int32  userID
	int32  robotID     # -1 all, 0 none, 107 specific
	int32  severity
	string displayMsg  # more verbose message, to display on screen
	string speakMsg    # if len(speakMsg) > 0, speak this string
	'''

	msg = audio_response()

	# msg.reqHostname = reqHostname
	msg.userID     = userID
	msg.robotID    = robotID
	msg.displayMsg = displayMsg
	msg.speakMsg   = speakMsg
	msg.severity   = severity
					
	pub_audio_response.publish(msg)


# ---------------------------------------------------------


class Main:
	def __init__(self):
		rospy.init_node('main', anonymous=True)
		
		# Set the shutdown function
		rospy.on_shutdown(self.shutdown)
		

		# Define publishers
		try:
			self.pub_cmd_vel           = rospy.Publisher('cmd_vel', Twist, queue_size=1)
			self.pub_audio_transcribed = rospy.Publisher('audio_transcribed', audio_transcribed, queue_size=1)
		except Exception as e:
			pubConsole(SEVERITY_ERROR, f'Error defining pub_cmd_vel: {e}')
			print()
		
			
		# Subscribe to basic/common topics:
		try:
			rospy.Subscriber('dpad',     dpad,     self.callback_dpad)
		except Exception as e:
			pubConsole(SEVERITY_ERROR, f'Error in subscribing to dpad: {e}')

		try:
			rospy.Subscriber('touchpad', touchpad, self.callback_touchpad)
		except Exception as e:
			pubConsole(SEVERITY_ERROR, f'Error in subscribing to touchpad: {e}')

		try:
			rospy.Subscriber('chat',     chat,     self.callback_chat)
		except Exception as e:
			pubConsole(SEVERITY_ERROR, f'Error in subscribing to chat: {e}')

		try:
			rospy.Subscriber('joystick', joystick,     self.callback_joystick)
		except Exception as e:
			pubConsole(SEVERITY_ERROR, f'Error in subscribing to joystick: {e}')

			
		self.t2s = None	
		try:
			self.t2s = ub_chat.Text2Speech()
			rospy.Subscriber('audio_command', audio_command, self.callback_audio_command)
		except Exception as e:
			pubConsole(SEVERITY_ERROR, f'Error in subscribing to audio_command: {e}')
			
					
		rospy.spin()
		
	def callback_audio_command(self, msg):
		txt = self.t2s.transcribe(audio_data=msg.data, initial_prompt=None, debug=False)

		# cleanup
		txt = ub_chat.cleanup(txt)

		# FIXME -- This is just for demo purposes.
		# See if we can make sense of the text to send a cmd_vel command:
		isMatch, directions = self.interpret(txt)
		if (isMatch):
			cmd = Twist()
			cmd.linear.x  = directions['x']
			cmd.angular.z = directions['y']
			self.pub_cmd_vel.publish(cmd)
		
		# Publish the cleaned transcribed text
		self.pubAudioTranscribed(msg.robotID, txt, msg.userID)
		
	def callback_chat(self, msg):
		print(msg)

		# cleanup
		txt = ub_chat.cleanup(msg.message)

		# FIXME -- This is just for demo purposes.
		# See if we can make sense of the text to send a cmd_vel command:
		isMatch, directions = self.interpret(txt)
		if (isMatch):
			cmd = Twist()
			cmd.linear.x  = directions['x']
			cmd.angular.z = directions['y']
			self.pub_cmd_vel.publish(cmd)
			
		
	def	callback_dpad(self, msg):
		print(msg)	
		# FIXME -- Translate to Twist command
		# You'll want to change the logic below...this is just for demo:
		cmd = Twist()
		if (msg.button == "stop"):
			# This is unnecessary, as `cmd = Twist()` already reset the values to zeros.
			cmd.linear.x  = 0
			cmd.angular.z = 0
		elif (msg.button == "forward"):
			cmd.linear.x = 1
		elif (msg.button == "back"):
			cmd.linear.x = -1
		elif (msg.button == "right"):
			cmd.linear.x  =  0
			cmd.angular.z = -0.1
		elif (msg.button == "left"):
			cmd.linear.x  = 0
			cmd.angular.z = 0.1
			
		self.pub_cmd_vel.publish(cmd)	

	def	callback_joystick(self, msg):
		print(msg)	
		# FIXME -- Translate to Twist command
		# Each joystick has different mappings 
		# of axes and/or buttons.
		# This example is just for demo.
		# MAKE SURE TO CHANGE THE VALUES.
		
		if (msg.controllerID == 0):
			# We'll only listen to this controllerID
			cmd = Twist()
	
			# Apply a "dead zone" around axis origins:
			if (abs(msg.axes[1]) > 0.02):
				cmd.linear.x  = -msg.axes[1]
			if (abs(msg.axes[0]) > 0.02):
				cmd.angular.z = -msg.axes[0]

			self.pub_cmd_vel.publish(cmd)	


	def callback_touchpad(self, msg):
		print(msg)
		# FIXME -- Translate to Twist command
		# Here's a sample:
		cmd = Twist()
		for info in msg.touchpadInfo:
			if (info.touchpadName == "tp1"):
				# In this example, we'll only listen to touchpad "tp1"
				# Note: the touchpad axes are cartesian (+y is up, +x is right).
				# You'll want to change the logic here...this is just for demo:
				cmd.linear.x =   info.y
				cmd.angular.z = -info.x/10 

				self.pub_cmd_vel.publish(cmd)	


	def pubAudioTranscribed(self, robotID, txt, userID):
		msg = audio_transcribed()

		msg.userID  = userID
		msg.robotID = robotID
		msg.txt     = txt
						
		self.pub_audio_transcribed.publish(msg)
		


		
	def interpret(self, txt):
		'''
		This is a demo function to show how 
		"regular expressions" might be used to
		parse some text and look for commands.
		There are other/better ways to do this...
		this is just an example to show 
		how chat-to-command works.
		'''
		foundCmd = False
		isMatch, directions = isMove(txt)
		
		return isMatch, directions
		
			
	def shutdown(self):
		# Gracefully shut things down.
		# Close files/processes, end loops, etc.
		pubConsole(SEVERITY_INFO, 'shutting down')

if __name__ == "__main__":
	Main()
