# Usage

`cd rosbag2video_docker`

`docker build -t rosbag2video:humble .`

`./run_docker.sh`

`./ros2bag2video.py [--fps 25] [--rate 1.0] [-o outputfile] [-v] [-s] [-t topic] data/bagname`

### store bags in data/ so they are accessible in container
