#!/bin/bash

# Name of the Docker image
IMAGE_NAME="rosbag2video:humble"

# Name of the Docker container
CONTAINER_NAME="rosbag2video"

# Path to the directory you want to mount inside the container
HOST_BASE_PATH="$(pwd)"
CONTAINER_BASE_PATH="/home/humble"

# Check if the container is already running
if [ $(docker ps -q -f name=${CONTAINER_NAME}) ]; then
  echo "Container ${CONTAINER_NAME} is already running."
else
  # Run the container
  docker run --rm -it --name ${CONTAINER_NAME} \
             -v ${HOST_BASE_PATH}/data:${CONTAINER_BASE_PATH}/data \
             -v ${HOST_BASE_PATH}/rosbags_video.py:${CONTAINER_BASE_PATH}/rosbags_video.py \
             -v /media:/media \
             ${IMAGE_NAME} /bin/bash
fi
