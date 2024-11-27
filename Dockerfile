FROM ros:humble-ros-base

RUN apt-get update && apt-get install --no-install-recommends -y \
    ffmpeg \
    git \
    python3-pip \
    python3-roslib \
    python3-sensor-msgs \
    python3-opencv \
    python3-setuptools \
    ros-humble-vision-opencv \
    ros-humble-rosbag2-transport \
    ros-humble-rosbag2-storage-mcap \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

RUN pip install rosbags opencv-python tqdm
    
WORKDIR /home/humble

CMD ["bash", "-c", "source /opt/ros/humble/setup.bash"]