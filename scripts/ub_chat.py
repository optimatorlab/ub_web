#!/usr/bin/env python3

from datetime import datetime 
import os
import re
import rospy
from ub_common import *


MIC_RECORD_DIRECTORY      = '{}/tmp/ub_web_mic'.format(HOME_DIRECTORY)


def cleanup(txt):
	txt = txt.lower()
	
	# Cleanup
	# * Remove leading/trailing whitespace
	txt = txt.strip()
	
	# * Remove any trailing punctuation
	txt = txt.rstrip('.')
	
	# * Replace '-' with ' '.  Ex:  '1-1-3' becomes '1 1 3', '1-3-9-er' becomes '1 3 9 er'
	txt = txt.replace('-', ' ')
	
	# * Replace spaces between consecutive numbers.  Ex:  '1 1 3' becomes '113', '1 3 9 er' becomes '139 er'
	txt = re.sub(r'(?<=[0-9./])\s+(?=[0-9./])','',txt)

	# * Make sure there's a space between a digit and a letter.  Ex: '3rd' becomes '3 rd'
	# TODO?
		
	# * Don't allow a leading period before an integer (i.e., replace .3 by 0.3)
	txt.replace(r' .', ' 0.')
	
	# * Replace "point" or "dot" with "."?.  Ex:  '1 point 3' becomes '1.3'
	(isMatch, substring) = processSearch(r'\d*\s*((\bpoint\b)|(\bdot\b))\s*\d+', txt)
	if (isMatch):
		newString = re.sub(r'\s*[a-z]+\s*', '.', substring)
		txt = txt.replace(substring, newString)
		
	# * Remove commas?
	txt = txt.replace(',', ' ')
	
	# Text to digits
	txt = re.sub(r'\bzero\b',  '0', txt)
	txt = re.sub(r'\bone\b',   '1', txt)
	txt = re.sub(r'\btwo\b',   '2', txt)
	txt = re.sub(r'\bthree\b', '3', txt)
	txt = re.sub(r'\bfour\b',  '4', txt)
	txt = re.sub(r'\bfive\b',  '5', txt)
	txt = re.sub(r'\bsix\b',   '6', txt)
	txt = re.sub(r'\bseven\b', '7', txt)
	txt = re.sub(r'\beight\b', '8', txt)
	txt = re.sub(r'\bnine\b',  '9', txt)
	
	return txt

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

def processSearchBeforeAfter(regex, text):
	# If regex is found anywhere in text, return the content before and after the match
	srch = re.search(regex, text)
	if (srch):
		substring = srch.string[srch.start():srch.end()].strip()
		a = text.split(substring)
		return (True, a[0].strip(), a[-1].strip())
	else:
		return (False, '', '')

		
class Text2Speech:
	def __init__(self, model="tiny.en"):
		try:
			import whisper  # (for some reason this has to be loaded first???)
	
			self.whisperModel = whisper.load_model(model)
		
			# Need to "prime" the model once so it goes fast the 
			# first time we actually want to use it:
			result = self.whisperModel.transcribe("../config/cancel_clip.wav", fp16=False)
			
			# Create MIC_RECORD_DIRECTORY directory, if it doesn't already exist
			if not os.path.exists(MIC_RECORD_DIRECTORY):
				os.makedirs(MIC_RECORD_DIRECTORY, exist_ok=True)
			
		except Exception as e:
			pubConsole(SEVERITY_ERROR, f'Error in Text2Speech init: {e}')
			
	
	def transcribe(self, audio_data, initial_prompt=None, debug=False):
		# NOTE:  Don't use "write()" to save .wav files.
		#        It's adding weird content to the files.
		# Instead, just output a binary file.
		
		try:
			myTimestamp = datetime.today()	
			filename = '{}/{}.wav'.format(MIC_RECORD_DIRECTORY, myTimestamp.strftime('%Y-%m-%d-%H%M%S'))

			# write audio data to file
			newFile = open(filename, "wb")
			newFile.write(bytearray(audio_data))

			if (initial_prompt is None):
				initial_prompt = "hey 107 you are a drone. i might command robot 1001 to yaw or face north.  hey 111 start aruco."
				
			# Transcribe
			# https://platform.openai.com/docs/api-reference/audio
			result = self.whisperModel.transcribe(filename, fp16=False, initial_prompt=initial_prompt)   # suppress_tokens=["y'all"]
			
			if (debug):
				# Write results to a file (for debugging)	
				fh = open("{}/mic.log".format(MIC_RECORD_DIRECTORY), "a")
				fh.write(f'{filename} | {result["text"]}\n')
				fh.close()
			else:
				# Delete the temporary audio file
				os.remove(filename)
				
			return result['text']
			
		except Exception as e:
			pubConsole(SEVERITY_ERROR, f"Error in transcribe: {e}")

