class Camera  {
	constructor(streamType='ros', topicName='/realsense/color/image_raw/compressed', 
				msgType='sensor_msgs/CompressedImage', 
				ip=HOST_IP, port=8000)  {
		this.createCameraDiv(streamType, topicName, msgType, ip, port);
		// this.createButtonFIXME();
	}
	
	createCameraDiv(streamType, topicName, msgType, ip, port)  {
		// Outer camera div
		var newDiv = document.createElement("div");
		newDiv.setAttribute("id", "divCamera");

		newDiv.style.position    = 'absolute';
		newDiv.style.top         = 0;
		newDiv.style.width       = '100%'; 
		newDiv.style.height      = '100%';
		// newDiv.style.height     = '30px';
		newDiv.style.marginLeft  = 'auto';
		newDiv.style.marginRight = 'auto';
		newDiv.style.textAlign   = 'center';
		newDiv.style.zIndex      = 0;		
		newDiv.style.display     = 'inline-block';
		newDiv.style.backgroundColor = 'gray';
		divMain.appendChild(newDiv);	


		var img = document.createElement("img");
		img.setAttribute("id", "visionStreamImg");
		img.style.maxWidth = "640px";
		img.style.maxHeight = '100%';
		divCamera.appendChild(img);
		img.onerror = this.standby(visionStreamImg);

		if (streamType == 'ros')  {
			this.createROSstream(topicName, msgType);			
		}  else  {
			this.createHTTPSstream(ip, port);			
		}
		
		/* FIXME 
		    - Add select list to choose/change resolution
		    - Add textbox to change FPS
			- Add button to take picture  pubActionGroupConfig("takePhoto", [])
			- Add menu to configure ArUco/ROI/etc.
			- [x] Add link to gallery
			- [x] Add gallery div (shows all the pics we've taken)
		*/
		
		// newDiv = document.createElement("div");
		
	}	
	
	createHTTPSstream(ip, port)  {
		visionStreamImg.src = "https://" + ip + ":" + port + "/stream.mjpg";
	}
	
	
	createROSstream(topicName, msgType)  {
		// Define a camera topic:	
		rostopic['CompressedImage'] = new ROSLIB.Topic({
			ros : ros,
			// name: 'camera/rgb/image_raw',
			// messageType : 'sensor_msgs/Image'
			// name: 'camera/rgb/image_raw/compressed', 
			name: topicName, // '/realsense/color/image_raw/compressed',
			messageType : msgType  // 'sensor_msgs/CompressedImage'
		});
		
		// Subscribe to the camera topic:
		rostopic['CompressedImage'].subscribe(function(message) {
			visionStreamImg.src = `data:image/png;base64,${message.data}`;
		});
	}
	

	setCfg(data, vehicleType)  {
		// Set cfg data after fetch is resolved.
		// Config data should be of the form:
		// "simTinybot": {
		// "motor": {"trim":     {"left":    {"value": 0, "inputtype": "number", "datatype": "float", "min": -1, "max": 1, "step": 0.05},  
		// 						"right":   {"value": 0, "inputtype": "number", "datatype": "float", "min": -1, "max": 1, "step": 0.05}},  
		// 			"throttle": {"min":     {"value": 0.4,  "inputtype": "number", "datatype": "float", "min":  0, "max": 1, "step": 0.05}},
		// 			"speed":    {"forward": {"value": 0.9,  "inputtype": "number", "datatype": "float", "min":  0, "max": 1, "step": 0.05}, 
		// 						"back":    {"value": 0.5,  "inputtype": "number", "datatype": "float", "min":  0, "max": 1, "step": 0.05},
		// 						"left":    {"value": 0.75, "inputtype": "number", "datatype": "float", "min":  0, "max": 1, "step": 0.05},
		// 						"right":   {"value": 0.75, "inputtype": "number", "datatype": "float", "min":  0, "max": 1, "step": 0.05},
		// 						"stop":    {"value": 0,    "inputtype": "number", "datatype": "float", "min":  0, "max": 0, "step": 0}} },
		// "cameras": {"front": {"camModel": "simTinybot", "topic": "/robot/camera/image_raw/compressed", 
		// 						"res_rows": 480,  "res_cols": 640, "fps_target": 30, "outputPort": 8000, 
		// 						"intrinsics": {"640x480": {"cx": 320.5, "cy": 240.5, "fx": 277.191356, "fy": 277.191356, 
		// 												"dist": [0.0, 0.0, 0.0, 0.0, 0.0]} } } } }
		// Here, `vehicleType` is "simTinybot".
		
		this.cfg = data[vehicleType].cameras;
		console.log(this.cfg);
	}	
	
	standby(elem) {
		console.log('oops');
		elem.src = 'images/warning_yellow.png';
	}

	createButtonFIXME()  {
		var newDiv = document.createElement("div");		
		newDiv.setAttribute("id", "divTakePhoto");

		newDiv.style.position    = 'absolute';
		newDiv.style.bottom      = '60px';
		newDiv.style.width       = '100%'; 
		// newDiv.style.height     = '30px';
		newDiv.style.marginLeft  = 'auto';
		newDiv.style.marginRight = 'auto';
		newDiv.style.textAlign   = 'center';
		newDiv.style.zIndex      = 5;		
		// newDiv.style.display     = 'inline-block';
		divCamera.appendChild(newDiv);			
		
		newDiv.innerHTML='<button onClick="pubActionGroupConfig(\'takePhoto\', []);">Pic</button>';
	}
}
