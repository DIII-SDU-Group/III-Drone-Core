#!/bin/sh

set -x
set -e

SCRIPT_DIR=$(dirname $0)

# If $1 is not equal to "--drone" or "", print error
if [ "$1" != "--drone" ] && [ "$1" != "" ]; then
    echo "Invalid argument: $1"
    echo "Usage: ./install.sh [--drone]"
    echo
    echo "Options:"
    echo "  --drone: Install for drone platform"
    exit 1
fi

sudo apt install -y tmux tmuxinator

# mkdir -p ~/.config/tmuxinator/
# cp $SCRIPT_DIR/../tmuxinator/* ~/.config/tmuxinator/

# mkdir -p ~/.config/iii_drone/parameters/

# if [ ! -f ~/.config/iii_drone/parameters/parameters.yaml ]; then
#     cp $SCRIPT_DIR/../config/parameters.yaml ~/.config/iii_drone/parameters/parameters.yaml
# else
#     $SCRIPT_DIR/update_installed_parameters.py $SCRIPT_DIR/../config/parameters.yaml ~/.config/iii_drone/parameters/
# fi


# If $1 is equal to "--drone", install udev rules
if [ "$1" = "--drone" ]; then
    sudo cp $SCRIPT_DIR/../udev/99-diii-usb.rules /etc/udev/rules.d/
fi

# echo "Finished installation. Use the script update_installed_parameters.py to make future updates to the parameters"
