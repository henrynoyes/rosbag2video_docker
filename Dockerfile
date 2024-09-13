FROM ros:humble-ros-base

RUN apt-get update && apt-get install --no-install-recommends -y \
    ffmpeg \
    git \
    python3-pip \
    python3-roslib \
    python3-sensor-msgs \
    python3-opencv \
    python3-setuptools \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*
    
WORKDIR /home/humble

RUN git clone https://github.com/mlaiacker/rosbag2video.git

RUN chmod +x ./rosbag2video/ros2bag2video.py

CMD ["bash", "-c", "source /opt/ros/humble/setup.bash"]