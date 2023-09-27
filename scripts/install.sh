#!/bin/sh

set -x

SCRIPT_DIR=$(dirname $0)

sudo apt install -y tmux tmuxinator

mkdir -p ~/.config/tmuxinator/
cp $SCRIPT_DIR/../tmuxinator/* ~/.config/tmuxinator/
mkdir -p ~/.config/iii_drone
cp $SCRIPT_DIR/../config/params.yaml ~/.config/iii_drone/params.yaml
