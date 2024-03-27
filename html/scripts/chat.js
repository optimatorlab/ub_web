// Define the audio_command topic:
rostopic['audio_command'] = new ROSLIB.Topic({
    ros : ros,
    name : 'audio_command',
    messageType : 'ub_web/audio_command'
});

// Define a function to publish audio_command
async function pubAudioCommand(blob, fileFormat)  {
    new Response(blob).arrayBuffer().then(buffer=>{
        uint=[...new Uint8Array(buffer)];

		var vehicleID = -1;
		if (config.selectedVehicle != undefined)  {	
			vehicleID = parseInt(config.selectedVehicle);
		}
		
		var cmd = {userID: config.userID, robotID: config.robotID, format: fileFormat, data: uint};
		var myAudioCommandCmd = new ROSLIB.Message(cmd);
		rostopic['audio_command'].publish(myAudioCommandCmd);
    });
}	

// Define the chat topic.  We'll both publish and subscribe to this topic
rostopic['chat'] = new ROSLIB.Topic({
    ros : ros,
    name : 'chat',
    messageType : 'ub_web/chat'
});

function pubChat(el)  {
	if (el.value.length > 0)  {
		// Add most recent command to *front* of queue
		// Reset txtCmdIndex to -1 each time we submit a command.
		chat.txtCmdHistory.unshift(el.value);
		chat.txtCmdIndex = -1;

		var cmd = {userID: config.userID, message: el.value, privateRecipients: []};
		var myCmd = new ROSLIB.Message(cmd);
		rostopic['chat'].publish(myCmd);
		// console.log(myCmd);
		el.value = '';
	}
}


rostopic['chat'].subscribe(function(msg)  {
	cons.addToConsole(msg.message + '\n', -1, "chat", msg.userID);
});

rostopic['audio_transcribed'] = new ROSLIB.Topic({
	ros : ros,
	name : 'audio_transcribed',
	messageType : 'ub_web/audio_transcribed'	
});

rostopic['audio_transcribed'].subscribe(function(msg)  {
	if ((msg.userID < 0) || (msg.userID == config.userID))  {		
		if ((msg.userID == config.userID) && (msg.txt.length > 0))  {
			// userID is the one who recorded the message
			cons.addToConsole(msg.txt + '\n', -1, "audio_transcribed", msg.userID);		
		}  else  {
			console.log('FIXME.  Do we really need to consider this case?');
		}
	}	
});

rostopic['audio_response'] = new ROSLIB.Topic({
	ros : ros,
	name : 'audio_response',
	messageType : 'ub_web/audio_response'	
});

// Subscribe to the audio_response topic:
rostopic['audio_response'].subscribe(function(message) {
	if ((message.userID < 0) || (message.userID == config.userID))  {		
		// FIXME -- Apply logic
		// If no voices or if user asks for quiet, just print to div
		if (message.speakMsg.length > 0)  {
			cons.addToConsole(message.displayMsg + '\n', message.severity, "audio_response", message.userID);
			speak(message.speakMsg);
		}
	}
});    


// ==========================================================


class Chat  {
	constructor(parentDiv, args = {})  {
		/*
			location = {'top': , 'bottom': , 'left': , 'right': }.
		               Must include either 'top' or 'bottom'; either 'left' or 'right'
		*/
				
		if (args.sounds) {	
			this.sounds = args.sounds;
		}  else  {
			this.sounds = { micStart: new Audio('sounds/bell.oga'), 
					        micEnd:   new Audio('sounds/complete.oga') };
		}

		if (args.constraints)  {
			this.constraints = args.constraints;
		}  else  {
			this.constraints = { audio: {volume: 0.7, channels: 1, 
										 sampleRate: 16000, autoGainControl: false, 
										 echoCancellation: false, googAutoGainControl: false, noiseSuppression: true}, 
								 video: false };
		}

		if (args.maxMicRecordSec) {
			this.MAX_MIC_RECORD_MS = args.maxMicRecordSec * 1000;
		}  else  {
			this.MAX_MIC_RECORD_MS = 4 * 1000;	// milliseconds that we're willing to let the mic record audio before timeout
		}
		
		if (args.icons)  {
			this.icons = args.icons;
		}  else  {
			this.icons = {mic: {on:  'images/30_mic_on.png', 
								off: 'images/30_mic_off.png', 
								disabled: 'images/30_mic_disabled.png'}};		
		}		

		if (args.location)  {
			this.location = args.location;
		}  else  {		
			this.location={'top': '2px', 'right': '271px'};
		}

				
		// We'll set these below
		this.micRecordStart = null;
		this.micRecordStop  = null;
				
		this.options = null;
		this.fileFormat = null;
		this.chunks = [];
				
		// These were in `config`:
		this.micStartTimestamp = new Date(0);
		this.micRecording      = false;
		this.micDisabled       = false;
		
		// This gets officially set in `initMic()` below.					
		this.micCheckFunc = this.micCheck_pass;
					
		// Keep track of command history in txtChat textarea
		this.txtCmdHistory = [];
		this.txtCmdIndex   = 0;
							
		this.createDivMic(this.location);
		this.createDivButton();
		this.initMic();
		try {
			this.defineEventListenerMicKeyboard();
		} catch (err)  {
			console.log(err);
		}
		this.createDivChat(parentDiv);
		
		joystickTriggers = [{btn: 3, func: this.micCheck}];
	}
	
	createDivMic(location)  {		
		var newDiv = document.createElement("div");
		newDiv.setAttribute("id", "micIconDiv");
		newDiv.style.position = 'absolute';
		if ('left' in location)  {
			newDiv.style.left    = location.left;   // '10px';
		}  else  {
			newDiv.style.right   = location.right;  // '10px';			
		}
		if ('top' in location)  {  
			newDiv.style.top     = location.top;    // '10px';
		}  else  {
			newDiv.style.bottom  = location.bottom; // '10px';			
		}
		newDiv.style.width     = '20px'; 
		newDiv.style.textAlign = 'center';
		// newDiv.style.zIndex      = 1;		
		// newDiv.style.display     = 'inline-block';

		newDiv.innerHTML = '<div id="micIconDivImg" style="position:absolute;top:0;left:0;width:20px;height:20px;padding:3px;border-radius:8px;">' +
						   '    <img id="imgMicIcon" src="images/30_mic_off.png" style="width:20px;"/>' + 
						   '</div>';

		document.body.appendChild(newDiv);		
	}
	
	createDivButton()  {
		var newDiv = document.createElement("div");
		newDiv.style.position  = 'absolute';
		newDiv.style.right     = '15px';			
		newDiv.style.top       = '50px';
		newDiv.style.width     = '75px'; 
		newDiv.style.textAlign = 'center';
		
		newDiv.innerHTML = '<button class="prevent-select" style="color:white;background-color:#40e0d0;border:none;border-radius:5px;font-weight:bold;width:80px;height:50px;"' +
						   'onmousedown="chat.micCheck(1);" onmouseup="chat.micCheck(0);" ' +
						   'ontouchstart="chat.micCheck(1);" ontouchend="chat.micCheck(0);" title="ctrl+shift+enter">' +
						   'push to talk</button>';	

		document.body.appendChild(newDiv);		
	}
	
	createDivChat(parentDiv)  {
		var newDiv = document.createElement("div");
		newDiv.setAttribute("id", "chatDiv");
		
		newDiv.innerHTML = '<div style="height:50px;">' +
						   '<div style="float:left;width:80%;height:40px;margin:4px 0px;"><textarea id="txtChat" style="width:100%;height:30px;border-radius:10px;padding:5px;resize:none;" title="ctrl+arrows to view history"></textarea></div>' +
						   '<div style="float:right;"><img src="images/50_blue_arrow_up_send.png" height=40px style="cursor:pointer;margin-top:5px;margin-right:2px;" onclick="pubChat(txtChat);" title="ctrl+enter"></div>' +
						   '</div>';
		parentDiv.appendChild(newDiv);
	}

	defineEventListenerMicKeyboard()  {
		if (!navigator.userAgentData.mobile)  {
			// Assume we're on a laptop (with a keyboard)
			document.addEventListener('keyup', event => {
				/*
				if (!event.ctrlKey) {
					console.log('no ctrl');
					return;
				}
				*/
				// console.log('keyup ' + event.code);
				if (event.code == 'Space')  {
					this.micCheckFunc(false);
				} 
			});

			document.addEventListener('keydown', event => {
			  // Looking for ctrl+shift+spacebar to start mic recording
			  // Or ctrl+enter to publish chat window
			  if (!event.ctrlKey) { 
				return; 
			  } 
			  event.preventDefault();
	  		  // console.log('keydown ' + event.code);
			  if (event.code == 'Space') {
				this.micCheckFunc(true);
			  }  else if (event.code == 'Enter')  {
			  	pubChat(txtChat);
			  }  else if (event.code == 'ArrowUp')  {
				// ctrl + uparrow --> 
				if (document.activeElement === txtChat)  {
					this.txtCmdIndex += 1;
					this.txtCmdIndex = Math.min(this.txtCmdIndex, this.txtCmdHistory.length-1);
					if (this.txtCmdIndex < 0)  {
						txtChat.value = '';
					}  else  {
						txtChat.value = this.txtCmdHistory[this.txtCmdIndex];
					}
				}			  
			  }  else if (event.code == 'ArrowDown')  {
				// ctrl + downarrow	
				if (document.activeElement === txtChat)  {
					this.txtCmdIndex -= 1;
					this.txtCmdIndex = Math.max(this.txtCmdIndex, -1);
					if (this.txtCmdIndex < 0)  {
						txtChat.value = '';
					}  else  {
						txtChat.value = this.txtCmdHistory[this.txtCmdIndex];		
					}
				}			  
			  }
			  /*
			  switch (event.key) {
				case ' ' : doSomething('space'); break
				case 'z' : doSomething('z'); break;
				default : console.log('unhandled key was pressed');
			  }
			  */
			});
		}
	}	
	
	micCheck_pass(val)  {
		// Nothing to do here
		return;
	}
	
	micCheck(val)  {
		// Aliased by micCheckFunc, set in initMic() below.
		if (Boolean(val))  {
			if (!this.micRecording)  {
				if (!this.micDisabled)  {
					// Start recording
					this.micRecordStart();
				}
			}  else if (new Date() - this.micStartTimestamp > this.MAX_MIC_RECORD_MS)  {
				// Stop recording now
				this.micDisabled = true;
				this.micRecordStop();
			}
		}  else  {
			this.micDisabled = false;	
			if (this.micRecording)  {
				// Stop recording now
				this.micRecordStop();
			}
		}
	}
	
	initMic()  {
		try  {
			if (navigator.mediaDevices.getUserMedia) {
				// console.log('getUserMedia supported.');
				// FIXME -- Show white (enabled) mic icon on screen
				this.micCheckFunc = this.micCheck;

				let browser = getBrowser();    // "firefox", "chrome", "safari"
				if (browser == 'chrome')  {
					this.options = {mimeType: "audio/webm;codecs=pcm", audioBitsPerSecond:this.constraints.audio.sampleRate};   // 48000?  FIXME
					this.fileFormat = 'wav';  // 'pcm'
				}  else  {
					this.options = {mimeType: "audio/webm;codecs=opus", audioBitsPerSecond:this.constraints.audio.sampleRate};  // 48000?  FIXME
					this.fileFormat = 'webm'; // 'opus'
				}
							
				this.chunks = [];

				// I had some scope issues within the functions below.
				// `this` wasn't propogating all the way into the functions.
				// The hack below seems to be working.
				let obj = this;
				
				let onSuccess = function(stream) {
					const mediaRecorder = new MediaRecorder(stream, obj.options);
					
					// This will be called from gamepad trigger
					obj.micRecordStart = function() {
						obj.sounds.micStart.play();	
						
						obj.micStartTimestamp = new Date();
						obj.micRecording = true;
						
						document.getElementById('micIconDivImg').style.background = '#f40c0b';
						document.getElementById('imgMicIcon').src = this.icons.mic.on;
						
						mediaRecorder.start();
					}
									
					// This will be called from gamepad trigger
					obj.micRecordStop = function() {
						obj.micStartTimestamp = new Date();
						obj.micRecording = false;
						
						document.getElementById('micIconDivImg').style.background = '';
						document.getElementById('imgMicIcon').src = this.icons.mic.off;

						mediaRecorder.stop();

						obj.sounds.micEnd.play();
					} 

					mediaRecorder.onstop = function(e) {
						// console.log("data available after MediaRecorder.stop() called.");
						// const blob = new Blob(chunks, { 'type' : 'audio/ogg; codecs=opus' });
						// const blob = new Blob(chunks, { 'type' : 'audio/ogg; codecs=vorbis' });
						const blob = new Blob(obj.chunks, {'type': obj.options.mimeType});

						obj.chunks = [];

						pubAudioCommand(blob, obj.fileFormat);
					}

					mediaRecorder.ondataavailable = function(e) {
						obj.chunks.push(e.data);
					}
				}

				let onError = function(err) {
					console.log('The following error occured: ' + err);
				}

				navigator.mediaDevices.getUserMedia(this.constraints).then(onSuccess, onError);
			} else {
				alert('getUserMedia not supported on your browser');
				// Show gray (disabled) mic icon on screen
				document.getElementById('micIconDivImg').style.background = '';
				document.getElementById('imgMicIcon').src = this.icons.mic.disabled;
				
				this.micCheckFunc = this.micCheck_pass;
			}	
		}  catch (err) {
			alert('getUserMedia not supported on your browser:' + err);
			// Show gray (disabled) mic icon on screen
			document.getElementById('micIconDivImg').style.background = '';
			document.getElementById('imgMicIcon').src = this.icons.mic.disabled;
			
			this.micCheckFunc = this.micCheck_pass;			
		}	
	}
}	
	
	
