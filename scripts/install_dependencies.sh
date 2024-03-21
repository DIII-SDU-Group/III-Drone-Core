#!/bin/sh

set -x

behaviortree_cpp_url="https://github.com/DIII-SDU-Group/BehaviorTree.CPP.git"
behaviortree_cpp_tag="4.5.2"

SCRIPT_DIR=$(dirname $0)

ROS_VERSION=$(cat $SCRIPT_DIR/../.ROS_VERSION)

dependencies="
    ros-$ROS_VERSION-pcl-ros
    libopencv-dev
    python3-opencv
    ros-$ROS_VERSION-cv-bridge
    ros-$ROS_VERSION-image-transport
    ros-$ROS_VERSION-usb-cam
"

sudo apt update

sudo apt install -y $dependencies

# Install BT.CPP
cd $SCRIPT_DIR/../extern

# If folder BehaviorTree.CPP does not exist, clone the repository
if [ ! -d "BehaviorTree.CPP" ]; then
    git clone $behaviortree_cpp_url
fi

cd BehaviorTree.CPP
git checkout $behaviortree_cpp_tag

mkdir -p build
cd build
cmake ..
make
sudo make install