#!/bin/bash

# Export an environment variable
export GZ_SIM_RESOURCE_PATH=`pwd`/:
echo "MY_VARIABLE set to: $GZ_SIM_RESOURCE_PATH"


export GZ_SIM_SYSTEM_PLUGIN_PATH=`pwd`/plugins:
echo "MY_VARIABLE set to: $GZ_SIM_SYSTEM_PLUGIN_PATH"

# Launch an application (replace with your app)
APP_PATH= gz sim -v 4 `pwd`/model.sdf 


#if [ -f "$APP_PATH" ]; then
    echo "Launching application..."
    "$APP_PATH" 
else
    echo "Error: Application not found at $APP_PATH"
    exit 1
fi