#!/usr/bin/env bash

# Refresh .bashrc to get the appropriate/up-to-date ROS IP addresses
source ${HOME}/.bashrc

sleep 1s


# Export the ROS IP address to be used in Web page:
CURRENT_IP_FILE="${HOME}/catkin_ws/src/ub_web/html/scripts/currentIP.js"

echo "// Updated by ub-web-start, with hard-coded IP address" > $CURRENT_IP_FILE
echo "// ${ROS_HOSTNAME}" >> $CURRENT_IP_FILE
echo "var ROS_IP = 'wss://${ROS_HOSTNAME}:9090';" >> $CURRENT_IP_FILE
echo "var HOST_IP = '${ROS_HOSTNAME}';" >> $CURRENT_IP_FILE


# Start node.js web server
SCRIPT1="node server_secure.cjs --public"
# gnome-terminal --tab --title "NODE.JS" --working-directory=${HOME}/catkin_ws/src/ub_web/html -e "bash -ic \"export HISTFILE=${HOME}/.bash_history_junk1; $SCRIPT1; history -s $SCRIPT1; exec bash\"" 
gnome-terminal --tab --title "NODE.JS" --working-directory=${HOME}/catkin_ws/src/ub_web/html -- bash -ic "export HISTFILE=${HOME}/.bash_history_junk1; $SCRIPT1; history -s $SCRIPT1; exec bash" 

# Start ROS:
# bash -c "source /opt/ros/noetic/setup.bash && source /home/pi/catkin_ws/devel/setup.bash --extend && roslaunch ub_web ub_web.launch"
SCRIPT2="roslaunch ub_web ub_web.launch"
# gnome-terminal --tab --title "ROSLAUNCH" --working-directory=${HOME}/catkin_ws/src/ub_web/launch -e "bash -ic \"export HISTFILE=${HOME}/.bash_history_junk2; $SCRIPT2; history -s $SCRIPT2; exec bash\"" 
gnome-terminal --tab --title "ROSLAUNCH" --working-directory=${HOME}/catkin_ws/src/ub_web/launch -- bash -ic "export HISTFILE=${HOME}/.bash_history_junk2; $SCRIPT2; history -s $SCRIPT2; exec bash" 

# Give ROS a little time to get started
sleep 2s

# Start our main.py node:
SCRIPT3="rosrun ub_web main.py"
# gnome-terminal --tab --title "MAIN.PY" --working-directory=${HOME}/catkin_ws/src/ub_web/scripts -e "bash -ic \"export HISTFILE=${HOME}/.bash_history_junk3; $SCRIPT3; history -s $SCRIPT3; exec bash\"" 
gnome-terminal --tab --title "MAIN.PY" --working-directory=${HOME}/catkin_ws/src/ub_web/scripts -- bash -ic "export HISTFILE=${HOME}/.bash_history_junk3; $SCRIPT3; history -s $SCRIPT3; exec bash" 


# Start chromium
sleep 5s
# firefox "https://localhost:8080/"
# chromium "https://localhost:8080/"
chromium "https://${ROS_HOSTNAME}:8080/"


# Print some info to the screen
echo "ROS_HOSTNAME = ${ROS_HOSTNAME}"
echo "ROS_MASTER_URI = $ROS_MASTER_URI"
echo ""
echo "Visit https://${ROS_HOSTNAME}:8080"
