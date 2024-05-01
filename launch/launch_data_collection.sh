#!/bin/bash

# Generate a timestamp
timestamp=$(date +'%Y%m%d-%H%M%S')

# Run the launch file, passing the timestamp as an argument
roslaunch jetbot_gazebo data_collection.launch data_path:=$timestamp