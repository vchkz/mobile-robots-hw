#!/bin/bash

xhost +local:docker || true
XAUTH=/tmp/.docker.xauth
IMAGE_NAME=ghcr.io/atokagzx/moveit2_base:latest

ROOT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd )"
docker pull ${IMAGE_NAME} || echo "[ERROR] Failed to pull image, trying to use cached"
docker run -ti --rm \
      -e "DISPLAY" \
      -e "QT_X11_NO_MITSHM=1" \
      -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
      -v /dev:/dev \
      -e XAUTHORITY \
      -e "XDG_RUNTIME_DIR=${XDG_RUNTIME_DIR}" \
      -e "ROS_DOMAIN_ID=0" \
      -e FASTRTPS_DEFAULT_PROFILES_FILE=/root/config/fastdds.xml \
      -v ./configs/fastdds.xml:/root/config/fastdds.xml \
      -v $ROOT_DIR/ros_ws:/root/ros_ws \
      --net=host \
      --privileged \
      --name ros2_dev_container ${IMAGE_NAME} \
      "cd /root/ros_ws && zsh"
