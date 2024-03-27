// This code was borrowed from:
// https://developer.mozilla.org/en-US/docs/Web/API/Gamepad_API/Using_the_Gamepad_API  
// https://developer.mozilla.org/en-US/docs/Games/Techniques/Controls_Gamepad_API

var haveEvents   = 'ongamepadconnected' in window;
var controllers  = {};

// Specify low/high frequencies for rumble, based on severity.
// 1 --> low severity, 3 --> high severity
var rumbleFreq = { 'low':  { 1: 0.1, 2: 0.5, 3: 0.9 }, 
				   'high': { 1: 0.1, 2: 0.5, 3: 0.9 }}    

var pollJSgamepad = true;

var joystickTriggers = [];


// Define the dpad topic:
rostopic['joystick'] = new ROSLIB.Topic({
	ros : ros,
	name : 'joystick',
	messageType : 'ub_web/joystick'
});

// Define a function to publish joystick data
function pubJoystick(axes, buttons, userID=-1, controllerID=0)  {
	var cmd = {userID:       parseInt(userID), 
			   controllerID: parseInt(controllerID), 
			   axes:         axes, 
			   buttons:      buttons};
	var myCmd = new ROSLIB.Message(cmd);
	rostopic['joystick'].publish(myCmd);
}


function connecthandler(e) {
	addgamepad(e.gamepad);
}

function addgamepad(gamepad) {
	cons.addToConsole('Gamepad Connected', SEVERITY.INFO, "console", config.userID);
	controllers[gamepad.index] = gamepad;
	requestAnimationFrame(updateStatus);
}

function disconnecthandler(e) {
	cons.addToConsole('Gamepad Disconnected', SEVERITY.ALERT, "console", config.userID);
	removegamepad(e.gamepad);
}

function removegamepad(gamepad) {
	delete controllers[gamepad.index];
}
 
 
function rumble(msg)  {
	var controller = controllers[config.selectedJoystickID];	
	
	if (controller != undefined)  {
		if (controller && controller.vibrationActuator) {		
			controller.vibrationActuator.playEffect("dual-rumble", {
			  startDelay: 0,
			  duration: 200,
			  weakMagnitude: 1.0,
			  strongMagnitude: 1.0,
			});// .then(success, failure);
		} else if (!controller)  {
			document.getElementById('joystick_rumble_msg').innerHTML = 'No gamepad connected';
		} else if (!controller.vibrationActuator)  {			
			// document.getElementById('joystick_rumble_msg').innerHTML = 'No haptics support';
			// Make phone itself vibrate
			navigator.vibrate(200);
		}
	}

	/*
	if (controllers[config.selectedJoystickID] != undefined)  {
		document.getElementById('joystick_rumble_msg').innerHTML = 'fff';

		controller.vibrationActuator.playEffect("vibration", {
		  startDelay: 0,
		  duration: 200,
		  weakMagnitude: rumbleFreq['low'][msg.severity],
		  strongMagnitude: rumbleFreq['high'][msg.severity],
		});	
	}
	*/
}

function updateStatus() {
	if (!haveEvents) {
		scangamepads();
	}
	
	// Loop over each controller (there's probably only one)
	// Then loop over the axes and buttons
	Object.keys(controllers).forEach(function(key) {
		// console.log(key + " " + obj[key]);
		var axes = []
		var btns = []
		for (var axis=0; axis<controllers[key].axes.length; axis++)  {
			axes.push(controllers[key].axes[axis]);
		}
		for (var btn=0; btn<controllers[key].buttons.length; btn++)  {
			btns.push(controllers[key].buttons[btn].value);
		}
		pubJoystick(axes, btns, config.userID=-1, key);

		// Might want to call some function based on a button value.
		// For example, trigger the mic?   chat.micCheckFunc(button value);
		// var triggers = [{btn: 3, func: chat.micCheckFunc}]
		/*
		for (var i = 0; i < joystickTriggers.length; i++)  {
			joystickTriggers[i].func(btns[parseInt(joystickTriggers[i].btn)]);				
		}
		*/
	});	
	
		
	if (pollJSgamepad)  {
		setTimeout(() => {
			requestAnimationFrame(updateStatus);
		}, 150);		// FIXME -- 250?  
	}	
}

function scangamepads() {
	var gamepads = navigator.getGamepads ? navigator.getGamepads() : (navigator.webkitGetGamepads ? navigator.webkitGetGamepads() : []);
	for (var i = 0; i < gamepads.length; i++) {
	  if (gamepads[i]) {
		if (gamepads[i].index in controllers) {
		  controllers[gamepads[i].index] = gamepads[i];
		} else {
		  addgamepad(gamepads[i]);
		}
	  }
	}
}


window.addEventListener("gamepadconnected", connecthandler);
window.addEventListener("gamepaddisconnected", disconnecthandler);
