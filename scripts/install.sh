#!/bin/sh

set -x

SCRIPT_DIR=$(dirname $0)

sudo apt install -y tmux tmuxinator

mkdir -p ~/.config/tmuxinator/
cp $SCRIPT_DIR/../tmuxinator/* ~/.config/tmuxinator/

mkdir -p ~/.config/iii_drone/parameters/

if [ ! -f ~/.config/iii_drone/parameters/parameters.yaml ]; then
    cp $SCRIPT_DIR/../config/parameters.yaml ~/.config/iii_drone/parameters/parameters.yaml
fi

if [ ! -f ~/.config/iii_drone/ros_params.yaml ]; then
    cp $SCRIPT_DIR/../config/ros_params.yaml ~/.config/iii_drone/ros_params.yaml
fi

echo "Finished installation. Use the script update_installed_parameters.py to make future updates to the parameters"
