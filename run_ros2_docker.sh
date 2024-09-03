#!/bin/bash

# XSOCK=/tmp/.X11-unix

# docker run -it --rm \
#  --runtime=nvidia \
#  -e DISPLAY=:0 \
#  -v $XSOCK:$XSOCK \
#  -v $HOME/.Xauthority:/root/.Xauthority \
#  --privileged \
#  --net=host \
#  --gpus all \
#  --env="QT_X11_NO_MITSHM=1" \
#  ros2 "$@"

docker run -it \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    ros2 
    #rqt
export containerId=$(docker ps -l -q)
