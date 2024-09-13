# Usage

`cd rosbag2video_docker`

`git checkout noetic`

`docker build -t rosbag2video:noetic .`

`./run_docker.sh`

`./rosbag2video.py [--fps 25] [--rate 1.0] [-o outputfile] [-v] [-s] [-t topic] data/bagname`

### store bags in data/ so they are accessible in container
