#!/bin/sh

set -x
set -e

CWD=$(pwd)

SCRIPT_DIR=$(dirname $0)

sudo apt update

sudo apt install -y $(eval echo $(cat $SCRIPT_DIR/apt_dependencies.txt))

cd $CWD