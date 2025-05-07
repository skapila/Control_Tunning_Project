#!/bin/bash

# Launch an application (replace with your app)
APP_PATH=  /home/bot-lab/ardupilot/Tools/autotest/sim_vehicle.py -D -v ArduCopter -f JSON --add-param-file=/home/bot-lab/ardupilot_gazebo/config/gazebo-iris-gimbal.parm

$APP_PATH 

