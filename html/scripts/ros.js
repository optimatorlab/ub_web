// ROS Variables:
// --------------
// NOTE:  Variable `ROS_IP` is set in `currentIP.js`
var ros = new ROSLIB.Ros({
	url : ROS_IP
	// url : 'ws://localhost:9090'
	// url : 'ws://192.168.1.175:9090'
});
		

// ROS Functions:
// --------------
ros.on('connection', function() {
	console.log('Connected to websocket server.');
});

ros.on('error', function(error) {
	console.log('Error connecting to websocket server: ', error);
});

ros.on('close', function() {
	console.log('Connection to websocket server closed.');
	// I don't think this ever executes.
});
// --------------


var rostopic = {};

