ARG ROS2_DISTRO=jazzy

FROM osrf/ros:${ROS2_DISTRO}-desktop

# Need pygame
RUN apt update && apt install -y \
    python3-pygame \
    dbus-x11 \
    packagekit-gtk3-module \
    libcanberra-gtk3-0 \
    libxrender-dev \
    libx11-6 \
    libxext-dev \
    libxinerama-dev \
    libxi-dev \
    libxrandr-dev \
    libxcursor-dev \
    libxtst-dev \
    libsdl2-dev \
    apulse \
    hicolor-icon-theme \
    libasound2t64 \
    libglx-mesa0 \
    libgl1 \
    libgl1-mesa-dri \
    libpulse0 \
    && rm -rf /var/lib/apt/lists/*

ENV DISPLAY=:0

RUN dbus-uuidgen > /etc/machine-id

WORKDIR /2d_movement
COPY ./commander ./commander
COPY ./controller ./controller
COPY ./interfaces ./interfaces
COPY ./test_pygame.py ./test_pygame.py

ARG ROS2_DISTRO
RUN . /opt/ros/${ROS2_DISTRO}/setup.sh && \
    rosdep update && \
    rosdep install -i --from-path controller/src --rosdistro ${ROS2_DISTRO} -y && \
    colcon build

ENTRYPOINT ["/bin/bash"]
