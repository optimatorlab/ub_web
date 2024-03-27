class Dpad  {
	constructor(name, outputFunction='pubDpad', location={'bottom': '0px', 'right': '0px'}, hidden=false)  {
		/*
			name -- div becomes 'div' + name
			outputFunction -- **string** of the **name** of the function to call when a button is pressed.  See `pubDpad()` below.
			location = {'top': , 'bottom': , 'left': , 'right': }.
		               Must include either 'top' or 'bottom'; either 'left' or 'right'
		*/
		
		this.name = name;
		this.divName = 'div' + name;
		
		this.createDpadDiv(name, outputFunction, location, hidden);		
	}
	
	createDpadDiv(name, outputFunction, location, hidden)  {
		// Outer Dpad div
		
		var styleName = name + 'Btns';
		var style = document.createElement("style");
		style.type = 'text/css';
		style.innerHTML = '.' + styleName + '{' + 
							'text-align: center;' + 
							'width:      40px;' + 
							'cursor:     pointer;' +
							'}';
		document.getElementsByTagName('head')[0].appendChild(style);
		
		var newDiv = document.createElement("div");
		newDiv.setAttribute("id", "div" + name);

		newDiv.style.position    = 'absolute';
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
		// newDiv.style.width    = '100%'; 
		// newDiv.style.height   = '30px';
		newDiv.style.zIndex      = 1;	
		newDiv.style.display = hidden ? 'none' : 'inline-block';

		newDiv.innerHTML         = 
			'<table>' +
			'	<tr><td></td>' + 
			'       <td><img src="images/arrow_up.png" class="' + styleName + '" onClick="' + outputFunction + '(\'' + this.name + '\', \'forward\');"></td>' + 
			'       <td></td></tr>' +
			'	<tr><td><img src="images/arrow_left.png" class="' + styleName + '" onClick="' + outputFunction + '(\'' + this.name + '\', \'left\');"></td>' + 
			'       <td><img src="images/arrow_stop.png" class="' + styleName + '" onClick="' + outputFunction + '(\'' + this.name + '\', \'stop\');"></td>' + 
			'       <td><img src="images/arrow_right.png" class="' + styleName + '" onClick="' + outputFunction + '(\'' + this.name + '\', \'right\');"></td></tr>' +
			'	<tr><td></td>' + 
			'       <td><img src="images/arrow_down.png" class="' + styleName + '" onClick="' + outputFunction + '(\'' + this.name + '\', \'back\');"></td>' +
			'       <td></td></tr>' +
			'</table>';

		divMain.appendChild(newDiv);	
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


function tidyupDpad(parentObj, instArray)  {
	// A collection of things that we'll do after we initialize this widget
	
	// Add a header:
	parentObj.addToDiv('<h4>DPAD</h4>');
	
	for (var i=0; i<instArray.length; i++)  {
		// Add a hide/show button to the Settings icon
		parentObj.addToDiv('<button onClick="' + instArray[i] + '.toggleDiv();" style="width:150px;margin:4px;">Toggle ' + instArray[i] + '</button><br>');	
	}
}

// Define the dpad topic:
rostopic['dpad'] = new ROSLIB.Topic({
	ros : ros,
	name : 'dpad',
	messageType : 'ub_web/dpad'
});

// Define a function to publish dpad data
function pubDpad(dpadName, btn, userID=-1)  {
	var cmd = {dpadName: dpadName, userID: userID, button: btn};
	var myCmd = new ROSLIB.Message(cmd);
	rostopic['dpad'].publish(myCmd);
}

