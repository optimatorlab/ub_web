var SEVERITY     = { 'EMERGENCY': 0, 'ALERT': 1, 'CRITICAL': 2, 'ERROR': 3, 'WARNING': 4, 'NOTICE': 5, 'INFO': 6, 'DEBUG': 7, 'CLEAR': 10 };
var SEVERITY_MAP = { 0: 'EMERGENCY', 1: 'ALERT', 2: 'CRITICAL', 3: 'ERROR', 4: 'Warning', 5: 'Notice', 6: 'info', 7: 'debug', 10: 'All Clear' }; 


class GenericIconDiv {
	constructor(instName, parentDiv, iconOff, iconOn, iconHeight, iconTitle, iconStyle, divStyle) {
		if (instName == null)   {  console.error('instName cannot be null');  return;  }
		if (parentDiv == null)  {  console.error('parentDiv cannot be null');  return;  }
		
		this.instName    = instName;       // Instance name
		this.parentDiv   = parentDiv
		this.divName     = 'div' + instName;
		this.divIconName = 'div' + instName + 'Icon';
		this.iconName    = instName + 'Icon';
		this.iconOff     = iconOff;
		this.iconOn      = iconOn;
		this.iconHeight  = iconHeight;		// "30px"
		this.iconTitle   = iconTitle;
		this.iconStyle   = iconStyle;
		this.divStyle    = divStyle;
	}

	createIcon()  {
		// Create an Icon/link to open a floating div
		
		var newDiv = document.createElement("div");
		newDiv.setAttribute("id", this.divIconName);

		newDiv.style.position   = 'absolute';
		newDiv.style.top        = '0px'; 
		newDiv.style.right      = '0px';
		newDiv.style.zIndex     = 1;
		newDiv.style.height     = '45px';
		newDiv.style.width      = '45px';
		newDiv.style.cursor     = 'pointer';
		newDiv.style.background = 'rgba(42, 42, 42, 0.9)';

		// overwrite default style
		for (const [key, value] of Object.entries(this.iconStyle)) {
			// console.log(key, value);
			newDiv.style[key] = value;
		}

		newDiv.innerHTML = '<img src="' + this.iconOff + '"' + 
									  ' height=' + this.iconHeight + 
								      ' onClick="' + this.instName + '.toggleDiv();" ' + 
								      ' id="' + this.iconName + '"' +
								      ' title="' + this.iconTitle + '"' + 
								      ' style="position:absolute;top:0;bottom:0;right:0;left:0;margin:auto;">'; 
		this.parentDiv.appendChild(newDiv);
	}

	createDiv()  {		
		var newDiv = document.createElement("div");
		newDiv.setAttribute("id", this.divName);

		newDiv.style.position   = 'absolute';
		newDiv.style.top        = '42px'; 
		newDiv.style.right      = '30px';
		newDiv.style.zIndex     = 2;
		// newDiv.style.height     = '30px';
		// newDiv.style.cursor     = 'pointer';
		newDiv.style.background = 'rgba(42, 42, 42, 0.7)';
		newDiv.style.display    = 'none';
		newDiv.style.color      = 'white';
		newDiv.style.padding    = '4px';
		newDiv.style.maxHeight  = '100%';
		newDiv.style.overflow   = 'auto';
		newDiv.style.boxSizing  = 'border-box';
		newDiv.style.width      = '300px';
		
		// overwrite default style
		for (const [key, value] of Object.entries(this.divStyle)) {
			// console.log(key, value);
			newDiv.style[key] = value;
		}
		
		newDiv.innerHTML = ''; 
			
		this.parentDiv.appendChild(newDiv);	
	}

	addToDiv(html)  {
		document.getElementById(this.divName).innerHTML += html;
	}
	
	overwriteDiv(html)  {
		document.getElementById(this.divName).innerHTML  = html;		
	}
	
	_divShow()  {
		document.getElementById(this.divName).style.display = 'inline-block'
		document.getElementById(this.iconName).src = this.iconOn;		
	}
	
	_divHide()  {
		document.getElementById(this.divName).style.display = 'none'
		document.getElementById(this.iconName).src = this.iconOff;
	}	

	hideDivs()  {
		this._divHide();
	}
}

function fullscreenStart()  {
	const elem = document.documentElement;
	if (elem.requestFullscreen) {elem.requestFullscreen()}
}

function fullscreenStop()  {
	if (document.exitFullscreen) {document.exitFullscreen()}
}

function fullscreenToggle()  {
	if (document.fullscreenElement)  {
		// Already fullscreen
		fullscreenStop();
	}  else  {
		// 
		fullscreenStart();
	}
}

