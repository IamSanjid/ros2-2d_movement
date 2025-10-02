#!/bin/bash
. install/setup.bash
ros2 run controller body &
ros2 run controller camera &
ros2 run commander view
