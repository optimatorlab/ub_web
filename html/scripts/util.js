function isInt(value)  {
    var x = parseFloat(value);
    return !isNaN(value) && (x | 0) === x;
}

function isNonNegInt(n) {
    if (isInt(n))  {
        if (n >= 0)  {
            return true;
        }  else  {
            return false;
        }
    }  else  {
        return false;
    }
};

function isNumber(n) {
    return !isNaN(parseFloat(n)) && isFinite(n);
};

function isPosNumber(n) {
    if (isNumber(n))  {
        if (n > 0)  {
            return true;
        }  else  {
            return false;
        }
    }  else  {
        return false;
    }
};

function metersPer(unit)  {
	if (unit == 'm')  {
		return 1;
	}  else if (unit == 'cm')  {
		return 1/100;
	}  else if (unit == 'mm')  {
		return 1/1000;
	}  else if (unit == 'in')  {
		return 1/39.37;
	}
	
	return undefined;
}

	function getBrowser()  {
		// See https://stackoverflow.com/questions/9847580/how-to-detect-safari-chrome-ie-firefox-and-opera-browsers

		// Firefox 1.0+
		if (typeof InstallTrigger !== 'undefined')  {
			return "firefox";
		}

		// Chrome 1 - 79
		if (!!window.chrome && (!!window.chrome.webstore || !!window.chrome.runtime))  {
			return "chrome";
		}

		var test = function(regexp) {return regexp.test(window.navigator.userAgent)}
		if (test(/chrome|chromium|crios/i)) {
			return "chrome";
		}	

		// Safari 3.0+ "[object HTMLElementConstructor]" 
		if (/constructor/i.test(window.HTMLElement) || (function (p) { return p.toString() === "[object SafariRemoteNotification]"; })(!window['safari'] || (typeof safari !== 'undefined' && window['safari'].pushNotification)))  {
			return "safari";
		}

		/*
		// Opera 8.0+
		if ( (!!window.opr && !!opr.addons) || !!window.opera || navigator.userAgent.indexOf(' OPR/') >= 0 )  {
			return "opera";
		}			

		// Edge 20+
		if (!isIE && !!window.StyleMedia)  {
			return "edge";
		}

		// Edge (based on chromium) detection
		if (isChrome && (navigator.userAgent.indexOf("Edg") != -1))  {
			return "edgechrome";
		}

		// Blink engine detection
		if ((isChrome || isOpera) && !!window.CSS)  {
			return "blink";
		}
		*/
		
		return "undefined";
	}



function getOS()  {
	// See https://stackoverflow.com/questions/11219582/how-to-detect-my-browser-version-and-operating-system-using-javascript
	// See https://stackoverflow.com/questions/38241480/detect-macos-ios-windows-android-and-linux-os-with-js
	if (navigator.userAgent.indexOf("Android") != -1) return "Android";
	else if (navigator.userAgent.indexOf("Win") != -1) return "Windows";
	else if (navigator.userAgent.indexOf("like Mac") != -1) return "iOS";
	else if (navigator.userAgent.indexOf("Mac") != -1) return "Mac";
	else if (navigator.userAgent.indexOf("Linux") != -1) return "Linux";
	
	return "undefined";
}
	
