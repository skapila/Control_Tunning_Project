#!/bin/bash

# Launch an application (replace with your app)
APP_PATH=  sim_vehicle.py -D -v ArduCopter -f JSON --add-param-file=/home/samarth/Desktop/ardupilot/Tools/autotest/default_params/gazebo-iris.parm
$APP_PATH 

