# Usage

`cd rosbag2video_docker`

`docker build -t rosbag2video:humble .`

`./run_docker.sh`

`python3 rosbags_video.py [-h] [-t TOPIC] [-o OUTPUT_PATH] [--fps FPS] [--width WIDTH] [--height HEIGHT] data/bagname`

### store bags in data/ so they are accessible in container

### also mounts /media for data on external drives
