import os
import rospy

from ub_web.msg import console

HOME_DIRECTORY = os.environ['HOME']  	# '/home/student' 

SEVERITY_EMERGENCY = 0
SEVERITY_ALERT     = 1
SEVERITY_CRITICAL  = 2
SEVERITY_ERROR     = 3
SEVERITY_WARNING   = 4
SEVERITY_NOTICE    = 5
SEVERITY_INFO      = 6
SEVERITY_DEBUG     = 7
SEVERITY_CLEAR     = 10

pub_console = rospy.Publisher("console", console, queue_size=5)

def pubConsole(severity, msgtext, robotID=0, userID=0, speakMsg=''):
	try:
		c_msg          = console()
		
		c_msg.robotID  = robotID
		c_msg.userID   = userID
		c_msg.severity = severity
		c_msg.text     = msgtext
		c_msg.speakMsg = speakMsg
		
		print(c_msg)
		
		pub_console.publish(c_msg)
	except Exception as e:
		print(f"Error in pubConsole: {e}")
