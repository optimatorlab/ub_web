
/*
	hide/show dpad
	hide/show touchpad
	choose camera
	set userid?  set robotID?
*/
	

class Settings extends GenericIconDiv {
	constructor(instName   = null, 
				parentDiv  = null, 
				iconOff    = "images/icon_config_off.png",
				iconOn     = "images/icon_config_on.png",
				iconHeight = '30px',
				iconTitle  = "Console",  
				iconStyle  = {},  
				divStyle   = {},
				badgeStyle = {})  {
		super(instName, parentDiv, iconOff, iconOn, iconHeight, iconTitle, iconStyle, divStyle)
				
		this.createIcon();
		this.createDiv();

		// Add buttons to toggle fullscreen and refresh page
		this.addToDiv('<button onClick="fullscreenToggle();" style="width:150px;margin:4px;">Full Screen</button>');
		this.addToDiv('<br><button onClick="window.location.reload();" style="width:150px;margin:4px;">Refresh Page</button>');
		
		try { 
			if (navigator.userAgentData.mobile)  {
				fullscreenStart();
			}
		} catch (err) {
			console.log(err);
		} 
    }
    
	toggleDiv()  {
		if (document.getElementById(this.divName).style.display == 'none')  {
			closeDivs('nw');			
			this._divShow();
		}  else  {
			this._divHide();
		}
	}	
}    
    
