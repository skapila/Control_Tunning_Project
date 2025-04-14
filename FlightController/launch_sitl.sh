#!/bin/bash

# Export an environment variable
export GZ_SIM_RESOURCE_PATH=`pwd`/models:`pwd`/worlds:
echo "MY_VARIABLE set to: $GZ_SIM_RESOURCE_PATH"


export GZ_SIM_SYSTEM_PLUGIN_PATH=`pwd`/build:
echo "MY_VARIABLE set to: $GZ_SIM_SYSTEM_PLUGIN_PATH"

# Launch an application (replace with your app)
APP_PATH=  sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console
$APP_PATH 

