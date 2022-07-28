#! /bin/bash

if [ "$1" ]; then
    echo "limiting compiling thread to -j$1"
    NJ=$1

else
    NJ=$(nproc --all)
fi
echo "catkin_make -j$NJ"

# all
catkin_make -j$NJ -C ./catkin_ws

source catkin_ws/devel/setup.bash