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

# If $1 is equal to "--drone", install udev rules
if [ "$1" = "--drone" ]; then
    sudo cp $SCRIPT_DIR/../udev/99-diii-usb.rules /etc/udev/rules.d/
fi

