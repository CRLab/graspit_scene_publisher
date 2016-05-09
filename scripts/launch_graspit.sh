#!/bin/bash

export GRASPIT=$(rospack find graspit)/graspit_source
export GRASPIT_PLUGIN_DIR=$(dirname $(catkin_find libgraspit_scene_publisher.so))

rosrun graspit graspit -p libgraspit_scene_publisher -w robonautPinchGlass
