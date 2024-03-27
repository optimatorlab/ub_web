class Console extends GenericIconDiv {
	constructor(instName   = null, 
				parentDiv  = null, 
				iconOff    = "images/icon_console_off.png",
				iconOn     = "images/icon_console_on.png",
				iconHeight = '30px',
				iconTitle  = "Console",  
				iconStyle  = {},  
				divStyle   = {},
				badgeStyle = {})  {
		super(instName, parentDiv, iconOff, iconOn, iconHeight, iconTitle, iconStyle, divStyle)
		
		this.divBadgeName = 'div' + instName + 'Badge';
		this.badgeCounter = 0;
		
		this.createIcon();
		this.createDiv();
		this.createBadge();

		this.divNameInner = 'div' + instName + 'Inner';
		var style = document.createElement("style");
		style.type = 'text/css';
		style.innerHTML = '.' + instName + 'Inner {' + 
						  '  border:           1px solid #00000069;' +
						  '  border-radius:    3px;' +
						  '  padding:          2px;' +
						  '  height:           90px;' +
						  '  overflow-x:       hidden;' +
						  '  overflow-y:       auto;' +
						  '  color:            #fff;' + 
						  '  font-size:        13px;' + 
						  '}';
		document.getElementsByTagName('head')[0].appendChild(style);
		this.addToDiv('<div id="' + this.divNameInner + '" class="' + instName + 'Inner"></div>');

		// Redirect javascript errors to our console
		var obj = this;
		window.onerror = function(message, source, lineno, colno, error) {
			obj.addToConsole('JAVASCRIPT ERROR: ' + error + ' (' + source + ', line ' + lineno + ')', 3, "console", config.userID);
		}
    }


	addToConsole(txt, severity, msgType, userID)  {
		/*
		txt -- string to print to console
		severity -- see SEVERITY_MAP
		msgType -- "console", "chat", "audio_command", or "audio_response"
		userID -- ID of user who sent the chat (does not apply to "console").  Used to format console colors.
		*/
		
		// var txt = SEVERITY_MAP[severity] + ": " + txt + "\n";

		var newDiv = document.createElement("div");
		newDiv.innerHTML = txt;
			
		if (msgType == 'console')  {
			newDiv.classList.add("severity_" + severity);
		}  else if (msgType == 'chat')  {
			// FIXME -- Add line for author/sender name
			if (userID == config.userID)  {
				newDiv.classList.add("chat_own");
			}  else  {
				newDiv.classList.add("chat_other");
			}
			// FIXME -- Allow option to hide/show chat messages (like filtering info messages, above).
		}  else if (msgType == 'audio_transcribed')  {
			// Assuming that this text is from our user:
			newDiv.classList.add("audio_own");
		}  else if (msgType == 'audio_response')  {
			// FIXME -- Add line for author/sender name
			// This will probably just be Mission Control
			newDiv.classList.add("audio_response");
		}
				
		// Add content
		document.getElementById(this.divNameInner).appendChild(newDiv);	
		this.scrollToBottom();
			
		// Display badge?
		if (document.getElementById(this.divName).style.display == 'none')  {
			this.badgeCounter += 1;
			document.getElementById(this.divBadgeName).innerHTML = this.badgeCounter;
			document.getElementById(this.divBadgeName).style.display = 'inline-block';
		}
	}

	/*
	I don't think this is used anywhere?
	addMessage(msg) {
		var txt = SEVERITY_MAP[msg.severity] + ": " + msg.text + "\n";

		var newDiv = document.createElement("div");
		newDiv.innerHTML = txt;
		newDiv.classList.add("severity_" + msg.severity);
	
		// Add content and scroll to bottom
		this.divNameInner.appendChild(newDiv);	
		this.divNameInner.scrollTop = this.divNameInner.scrollHeight	
	
		// Display badge?
		if (document.getElementById(this.divName).style.display == 'none')  {
			this.badgeCounter += 1;
			document.getElementById(this.divBadgeName).innerHTML = this.badgeCounter;
			document.getElementById(this.divBadgeName).style.display = 'inline-block';
		}
	}
	*/
	
	// FIXME -- Hide/Show INFO messages
	toggle_visibility(className) {
		elements = document.getElementsByClassName(className);
		for (var i = 0; i < elements.length; i++) {
			elements[i].style.display = elements[i].style.display == 'none' ? 'block' : 'none';
		}
	}
	

	createBadge()  {
		// Create a badge to sit on the icon
		
		var newDiv = document.createElement("div");
		newDiv.setAttribute("id", this.divBadgeName);

		newDiv.style.position     = 'relative';
		// newDiv.style.top        = '62px'; 
		// newDiv.style.left       = '0px';
		newDiv.style.float        = 'right';
		newDiv.style.zIndex       = 2;
		// newDiv.style.height       = '10px';
		// newDiv.style.cursor     = 'pointer';
		newDiv.style.background   = 'red';
		newDiv.style.color        = 'white';
		newDiv.style.borderRadius = '20px';
		newDiv.style.width        = '15px';
		newDiv.style.height       = '15px';
		newDiv.style.padding      = '4px';
		newDiv.style.fontWeight   = 'bold';
		newDiv.style.textAlign    = 'center';
		newDiv.style.display      = 'none';
				
		newDiv.innerText = ''; 
		document.getElementById(this.divIconName).appendChild(newDiv);
	}
	
	scrollToBottom()  {
		document.getElementById(this.divNameInner).scrollTop = document.getElementById(this.divNameInner).scrollHeight;		
	}
	
	toggleDiv()  {
		if (document.getElementById(this.divName).style.display == 'none')  {
			closeDivs('nw');
			this._divShow();
			document.getElementById(this.divBadgeName).style.display = 'none';
			this.badgeCounter = 0;
			this.scrollToBottom();
		}  else  {
			this._divHide();
		}
	}	
}









