# A Web Interface for Robots - IE 482/582

This repository contains code for a ROS-enabled webpage.  This is not a polished product; it's designed to provide some starter code to highlight these types of interactivity:
- "Touchpad" controls on mobile devices.  These can be used, for example, to control drones.
- "Direction Pad" controls (on mobile and desktop).  These are discrete buttons, like "move forward" or "stop".
- Support for gamepads (like XBox or PS5 controllers).  Connect your gamepad to your phone/computer and press a button.  The gamepad should be instantly recognized.
- Support for speech-to-text via the Whisper API.  This has been successfully tested on Android and Ubuntu devices.  It fails to work on Mac/iOS devices.  I don't do Windows.
- Video streams from ROS compressed image topics, or mjpg streams.
- A "console" to display messages.  The system can be run on a network with multiple users in "chat" mode.

![image](https://github.com/optimatorlab/ub_web/assets/18486796/2fc5222f-4ca8-4c0c-aaad-3043c5e6d339)


The idea is to make it "easy" for students to control robots via
- Physical external gamepad/joystick (e.g., an XBox controller)
- Voice ([Whisper speech-to-text](https://github.com/openai/whisper))
- Text
- Webpage buttons/sliders

Although the backend (web server and ROS components) must be on a ROS-capable computer (we use Ubuntu 20.04 at the moment), the end user just needs a web browser.



## Installation

**NOTE: This has only been tested on ROS Noetic running on Ubuntu 20.04, with Chromium/Chrome web browers.**
- The instructions below assume that you have already installed ROS Noetic on Ubuntu 20.04/

### Install [Whisper](https://github.com/openai/whisper), the OpenAI speech-to-text model.
This is optional, but is highly recommended.  It is free, no API-key, and runs locally on your machine.

As of the moment this file was written, `v20231117` was the latest version.  You may wish to experiment with a newer version, should one be released in the future.
```
pip install openai-whisper==20231117
```

You'll also need `ffmpeg`.  If it's not already installed on your machine:
```
sudo apt update && sudo apt install ffmpeg
```

### Clone the contents of the `ub_web` repo into `~/catkin_ws/src` 

```
cd ~/catkin_ws/src
git clone https://github.com/optimatorlab/ub_web.git
```

### Make the `ub_web` package
```
cd ~/catkin_ws
catkin_make
```

### Install node.js
```
cd ~/catkin_ws/src/ub_web/html
curl -sL https://deb.nodesource.com/setup_18.x | sudo -E bash -
sudo apt-get install -y nodejs
npm install
```

--- 

## Running

It's best if you start your robot first. This gives you a chance to make note of any video topics/streams that you might want to observe.  For example, we're going to start a simulated Husky UGV in Gazebo:
```
roslaunch husky_gazebo husky_playpen.launch
```
- Our Husky is publishing compressed image data to `/realsense/color/image_raw/compressed`. 

---

In a text editor, open `~/catkin_ws/src/html/index.html` and find the `var camera = new Camera(`... line.  Edit the topic name as appropriate.
- For our Husky, the line would read: `var camera = new Camera('ros', '/realsense/color/image_raw/compressed', 'sensor_msgs/CompressedImage', HOST_IP, 8000);`.

---

The `ub_web` package uses an odd combination of a `bash` shell script and a ROS launch file.  

```
cd ~/catkin_ws/src/ub_web
./ub-web-start.sh
```

This will spawn 3 terminal windows:
1. `node.js` -- This is our self-hosted web server
2. `roslaunch ub_web ub_web.launch` -- This starts `rosbridge_websocket`
3. `rosrun ub_web main.py` -- This is a simple node that subscribes to the topics published by the webpage. This node also manages `whisper`.

The `bash` script will also open a Chromium tab.


---

## Customization
- TODO:
    - Allow user to change the camera topic in the browser (instead of editing `index.html`).
    - Assign joystick buttons to different purposes.
    - Add "take picture" functionality (and save photos to a "gallery").
    - Display odometry info.
    - Display laserscan data.

---

## Limitations/Opportunities
- The webpage does not work with Mac/iOS devices.
- It also does not work with Firefox.
- The documentation is practically non-existent.  
- There's currently no support for UDP video.  I suspect that would be super easy to implement (good first issue).
- Some things are "easily" customizable, many are not.  A more modular system is possible with just a little cleanup.
