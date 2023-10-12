#!/bin/sh

set -x

SCRIPT_DIR=$(dirname $0)

ROS_VERSION=$(cat $SCRIPT_DIR/../.ROS_VERSION)

dependencies="
    ros-$ROS_VERSION-pcl-ros
    libopencv-dev
    python3-opencv
    ros-$ROS_VERSION-cv-bridge
    ros-$ROS_VERSION-image-transport
"

sudo apt update

sudo apt install -y $dependencies