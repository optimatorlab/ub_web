var TOUCHPAD = {MAX_QUIET_COUNT: 5,
				LOOP_DELAY_FAST: (1/5) * 1000,   // Convert Hz to milliseconds
				LOOP_DELAY_SLOW:     2 * 1000}   // Convert Hz to milliseconds



if (('ontouchstart' in window) || (navigator.maxTouchPoints > 0) || (navigator.msMaxTouchPoints > 0))  {
	// This is a touch device 
	console.log("This is a touch device");
}  else  {
	// This is not touch device
	console.log("FIXME -- Need to warn user that things aren't going to work");
}


class Touchpad  {
	constructor(name, width, height, location, hidden=false)  {
		/*  FIXME
		 * name  
		 * width
		 * height
		 * position = {'top': , 'bottom': , 'left': , 'right': }.
		 *            Must include either 'top' or 'bottom'; either 'left' or 'right'
		*/ 
		
		this.name    = name;
		this.divName = 'div' + name;
		this.bgCanvasID = 'cnv' + name + 'Bkgnd';
		this.fgCanvasID = 'cnv' + name;
				
		
		this.MAX_X    = parseInt(width);
		this.MAX_Y    = parseInt(height);	
		this.MIN_X    =   0;
		this.MIN_Y    =   0;
		this.MID_X    = (this.MAX_X - this.MIN_X)/2;
		this.MID_Y    = (this.MAX_Y - this.MIN_Y)/2;
		this.SPREAD_Y = (this.MAX_Y - this.MIN_Y);		


		this.TWO_PI = 2 * Math.PI;


		// Initialize the circle position:
		this.pos = {'x': 0, 'y': 0};


		this.createDiv(width, height, location, hidden);
				
		this.bgCanvasEl = document.getElementById(this.bgCanvasID);
		this.fgCanvasEl = document.getElementById(this.fgCanvasID);


		this.ctxt = {};
		this.ctxt['back']  = this.bgCanvasEl.getContext('2d');
		this.ctxt['front'] = this.fgCanvasEl.getContext('2d');

		this.drawAxes();        // Axes drawn on background canvas
		this.drawCircle();		// circle goes on front canvas
				
		this.ongoingTouches = [];		
		this.fgCanvasEl.addEventListener("touchstart",  this.handleStart.bind(this),  false);
		this.fgCanvasEl.addEventListener("touchend",    this.handleEnd.bind(this),    false);
		this.fgCanvasEl.addEventListener("touchcancel", this.handleCancel.bind(this), false);
		this.fgCanvasEl.addEventListener("touchmove",   this.handleMove.bind(this),   false);
	}
	
	
	createDiv(width, height, location, hidden)  {
		var newDiv = document.createElement("div");
		newDiv.setAttribute("id", this.divName);

		newDiv.style.position     = 'absolute';
		newDiv.style.background   = 'rgba(42,42,42,0.4)';
		newDiv.style.padding      = '0px';
		newDiv.style.borderRadius = '4px';
		newDiv.style.width        = String(width) + 'px'; // '150px';
		newDiv.style.height       = String(height) + 'px'; // '150px';
		if ('left' in location)  {
			newDiv.style.left     = location.left;  // '10px';
		}  else  {
			newDiv.style.right    = location.right;  // '10px';			
		}
		if ('top' in location)  {
			newDiv.style.top      = location.top;  // '10px';
		}  else  {
			newDiv.style.bottom   = location.bottom;  // '10px';			
		}
		newDiv.style.display = hidden ? 'none' : 'inline-block';
		newDiv.style.overflowY    = 'auto';
		newDiv.style.zIndex      = 4;	

		var parent = divMain;
		
		newDiv.innerHTML = 
			'<canvas id="' + this.bgCanvasID + '" width=' + width + ' height=' + height + ' style="position: absolute; left: 0; top: 0; margin: 0; padding: 0; z-index: 298;"></canvas>' +
			'<canvas id="' + this.fgCanvasID + '" width=' + width + ' height=' + height + ' style="position: absolute; left: 0; top: 0; margin: 0; padding: 0; z-index: 299;"></canvas>';
		parent.appendChild(newDiv);			
	}
	
  	drawAxes(xlabel=null, ylabel=null)  {	
		// Draw axes/crossbars on background canvas
		var context = 'back';
			
		this.ctxt[context].strokeStyle = "black";		
		this.ctxt[context].globalAlpha = 0.4;
		this.ctxt[context].lineWidth = 10;
	
		// Draw y axis
		this.ctxt[context].beginPath();
		this.ctxt[context].moveTo(parseInt(this.MID_X), this.MIN_Y);
		this.ctxt[context].lineTo(parseInt(this.MID_X), this.MAX_Y);
		this.ctxt[context].stroke();		

		// Draw x axis
		this.ctxt[context].beginPath();
		this.ctxt[context].moveTo(this.MIN_X, parseInt(this.MID_Y));
		this.ctxt[context].lineTo(this.MAX_X, parseInt(this.MID_Y));
		this.ctxt[context].stroke();		
		
		// Label the axes?
		this.ctxt[context].font = "10pt sans-serif";
		this.ctxt[context].textBaseline = "middle"; 
		this.ctxt[context].strokeStyle = "white";
		this.ctxt[context].fillStyle = "white";
		this.ctxt[context].globalAlpha = 1.0;
		
		if (xlabel != null)  {
			this.ctxt[context].textAlign = "left";
			this.ctxt[context].fillText(xlabel, this.MIN_X, this.MID_Y+10);
		}
		if (ylabel != null)  {
			this.ctxt[context].textAlign = "right"; 
			this.ctxt[context].fillText(ylabel, this.MID_X-6, this.MIN_Y+10);
		}
	}
	
	
	drawCircle()  {
		// Draw a circle on the foreground (front) canvas
		var context = 'front';
		this.ctxt[context].clearRect(this.MIN_X, this.MIN_Y, this.MAX_X, this.MAX_Y);  	  	

  	  	// On the canvas, y increases as you go DOWN
  	  	var [x, y] = [this.MID_X + this.MID_X*this.pos.x, this.MID_Y - this.MID_Y*this.pos.y];
  	  	
		this.ctxt[context].globalAlpha = 1.0;
		this.ctxt[context].fillStyle = "#FF0000";
		this.ctxt[context].beginPath();
		this.ctxt[context].arc(x, y, 20, 0, this.TWO_PI);
		this.ctxt[context].fill();
  	}
	
	
	copyTouch({ identifier, pageX, pageY }) {
	  return { identifier, pageX, pageY };
	}
	
	ongoingTouchIndexById(idToFind) {
		for (var i = 0; i < this.ongoingTouches.length; i++) {
			var id = this.ongoingTouches[i].identifier;
		
			if (id == idToFind) {
				return i;
			}
		}
		return -1;    // not found
	}
		
	_handleStart(el, thisTouch)  {
		this.pos.y = (this.MID_Y - (thisTouch.pageY - $(el).offset().top)) / this.MID_Y;			
		this.pos.x = ((thisTouch.pageX- $(el).offset().left) - this.MID_X) / this.MID_X;				
	}
	
	handleStart(evt)  {		
		evt.preventDefault();
		var touches = evt.changedTouches;
		
		if (this.ongoingTouches.length == 0)  {
			for (var i = 0; i < touches.length; i++) {
				this.ongoingTouches.push(this.copyTouch(touches[i]));
				
				this._handleStart(this.fgCanvasEl, touches[i]);
				this.drawCircle();
			}  
		}
	}

	_handleMove(el, thisTouch)  {
		// x is [-1,+1]
		var myX = ((thisTouch.pageX- $(el).offset().left) - this.MID_X) / this.MID_X;
		this.pos.x = Math.max(Math.min(1.0, myX), -1.0);	

		// y is [-1,+1]
		var myY = (this.MID_Y - (thisTouch.pageY - $(el).offset().top)) / this.MID_Y;
		this.pos.y = Math.max(Math.min(1.0, myY), -1.0);

		// console.log(this.pos);
	}

	handleMove(evt) {
		evt.preventDefault();
		var touches = evt.changedTouches;
				
		for (var i = 0; i < touches.length; i++) {
			var idx = this.ongoingTouchIndexById(touches[i].identifier);
			
			if (idx >= 0) { 
				this._handleMove(this.fgCanvasEl, touches[i]);
				
				this.drawCircle();		
			  	 
				this.ongoingTouches.splice(idx, 1, this.copyTouch(touches[i]));  // swap in the new touch record
			} else {
				// console.log("cant figure out which touch to continue");
			}
		}
	}

	
	handleEnd(evt) {
		evt.preventDefault();
		var touches = evt.changedTouches;
		
		for (var i = 0; i < touches.length; i++) {
			var idx = this.ongoingTouchIndexById(touches[i].identifier);
			
			if (idx >= 0) {	  
			  	// The user quit touching the screen. 
			  	// Return circle to default/center position.
				this.pos.x = 0.0;
				this.pos.y = 0.0;	
				
				this.drawCircle();
			  
				this.ongoingTouches.splice(idx, 1);  // remove it; we are done
			} else {
				// console.log("cant figure out which touch to end");
			}
		}
	}
	
	handleCancel(evt) {
		evt.preventDefault();
		var touches = evt.changedTouches;
		
		for (var i = 0; i < touches.length; i++) {
			var idx = this.ongoingTouchIndexById(touches[i].identifier);
			this.ongoingTouches.splice(idx, 1);  // remove it; we are done
		}
	}
	
	toggleDiv()  {
		if (document.getElementById(this.divName).style.display == 'none')  {
			document.getElementById(this.divName).style.display = 'inline-block'
		}  else  {
			document.getElementById(this.divName).style.display = 'none'
		}
	}	
}	
	
// ========================================

function touchpadLoop(tpads=[], quietCount=0, 
					  maxQuietCount=TOUCHPAD.MAX_QUIET_COUNT, 
					  loopDelayFast=TOUCHPAD.LOOP_DELAY_FAST, 
					  loopDelaySlow=TOUCHPAD.LOOP_DELAY_SLOW, 
					  loopDelay) {		
	setTimeout(function() {
		
		var keepLooping = true;
		var delay = loopDelayFast;
		var cmd = [];
		
		var isQuiet = true;
		// Loop over all touchpads
		for (var i=0; i<tpads.length; i++)  {
			var data = {touchpadName: tpads[i].name, x: parseFloat(tpads[i].pos.x.toFixed(2)), y: parseFloat(tpads[i].pos.y.toFixed(2)) };
			cmd.push(data);
			
			if ((data.x != 0) || (data.y != 0))  {
				isQuiet = false;
			}	
		}
		
		
		if (isQuiet)  {
			quietCount += 1;
			if (quietCount == maxQuietCount)  {
				console.log('quiet for too long.  Maybe issue "Hold" command?');
			}  else if (quietCount >= 3*maxQuietCount)  {
				// Exit out of loop?
				/*
				console.log('I give up.  Stopping touchpadLoop');
				keepLooping = false;
				*/
				
				// Slow the rate?
				// console.log('Slowing touchpadLoop to ' + loopDelaySlow + ' ms');
				delay = loopDelaySlow;					
			}	
		} else  {
			quietCount = 0;
		}
			
		if (quietCount < maxQuietCount)  {
			// console.log(cmd);
			pubTouchpad(cmd);
		}
		
		// run the loop again:
		if (keepLooping)  {
			touchpadLoop(tpads, quietCount, maxQuietCount, loopDelayFast, loopDelaySlow, delay);
		}
	}, loopDelay);
};
	

function tidyupTouchpad(parentObj, instArray)  {
	// A collection of things that we'll do after we initialize this widget
	
	// Add a header:
	parentObj.addToDiv('<h4>TOUCHPAD</h4>');
	
	for (var i=0; i<instArray.length; i++)  {
		// Add a hide/show button to the Settings icon
		parentObj.addToDiv('<button onClick="' + instArray[i] + '.toggleDiv();" style="width:150px;margin:4px;">Toggle ' + instArray[i] + '</button><br>');	
	}
}


// Define the touchpad topic:
rostopic['touchpad'] = new ROSLIB.Topic({
	ros : ros,
	name : 'touchpad',
	messageType : 'ub_web/touchpad'
});

// Define a function to publish touchpad data
function pubTouchpad(data, userID=-1)  {
	var cmd = {userID: userID, touchpadInfo: data};
	var myCmd = new ROSLIB.Message(cmd);
	rostopic['touchpad'].publish(myCmd);
}

	
