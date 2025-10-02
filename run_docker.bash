#!/bin/bash
xhost +local:docker \
&& docker run -it --rm \
    -e DISPLAY=:1 \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v ./:/2d_movement \
    --network=host \
    --privileged \
    2d_movement
