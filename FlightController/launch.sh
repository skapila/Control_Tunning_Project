#!/bin/bash

# Export an environment variable
export GZ_SIM_RESOURCE_PATH=/home/samarth/Desktop/ardupilot_gazebo/models:/home/samarth/Desktop/ardupilot_gazebo/worlds:
echo "MY_VARIABLE set to: $GZ_SIM_RESOURCE_PATH"

export GZ_SIM_SYSTEM_PLUGIN_PATH=/home/samarth/Desktop/ardupilot_gazebo/build:
echo "MY_VARIABLE set to: $GZ_SIM_SYSTEM_PLUGIN_PATH"

# Launch an application (replace with your app)
APP_PATH= gz sim -v4 -r iris_runway.sdf 
echo "Launching application..."
$APP_PATH 
