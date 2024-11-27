from pathlib import Path
import cv2
import argparse
from tqdm import tqdm
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore
from cv_bridge import CvBridge

parser = argparse.ArgumentParser(description='Convert ROS2 bag to MP4')
parser.add_argument('bag_path', type=str, help='input bag path')
parser.add_argument('-t', '--topic', default='/camera/camera/color/image_raw', type=str, help='image topic name')
parser.add_argument('-o', '--output-path', default='data/output.mp4', type=str, help='output mp4 path')
parser.add_argument('--fps', default=30, type=int, help='frames per second')
parser.add_argument('--width', default=640, type=int, help='image width')
parser.add_argument('--height', default=480, type=int, help='image height')

class VideoConverter:

    def __init__(self):

        self.typestore = get_typestore(Stores.ROS2_HUMBLE)
        self.bridge = CvBridge()
        self.fourcc =  cv2.VideoWriter_fourcc(*'mp4v')

    def handle_output(self, output_path):

        if Path(output_path).exists():
            print(f'Warning: {output_path} already exists')
            usrin = input('Do you want to overwrite it? (y/n)\n').strip().lower()

            while usrin not in ('y', 'n'):
                usrin = input("Inavlid input. Please enter 'y' or 'n'\n").strip().lower()

            if usrin == 'n':
                print('Avoiding overwrite. Exiting...')
                exit(0)
                
    def cv2_convert(self, msg, ts):

        encodings = {
            'rgb8': 'bgr8',
            'bgr8': 'bgr8',
            # '16UC1': '16UC1',
            # '32FC1': '32FC1'
        }

        try:
            if msg.encoding in encodings:
                img = self.bridge.imgmsg_to_cv2(msg, desired_encoding=encodings[msg.encoding])
                return img
            else:
                print(f'Unsupported image encoding: {msg.encoding}. Exiting...')
                exit(1)
        
        except Exception as e:
            print(f'Error converting image at {ts}: {e}')
            return None
        
    def process(self, bag_path, topic, output_path, fps, width, height):

        self.handle_output(output_path)
        video_writer = cv2.VideoWriter(output_path, self.fourcc, fps, (width, height))

        with AnyReader([Path(bag_path)], default_typestore=self.typestore) as reader:
            connections = [x for x in reader.connections if x.topic == topic]

            print(f'Reading {bag_path} ...')
            msg_count = sum(1 for _ in reader.messages(connections=connections))
            with tqdm(total=msg_count, desc='Processing', unit='frames') as pbar:
                for connection, timestamp, rawdata in reader.messages(connections=connections):
                    msg = reader.deserialize(rawdata, connection.msgtype)
                    img = self.cv2_convert(msg, timestamp)

                    if img is not None:
                        video_writer.write(img)
                    pbar.update(1)

        video_writer.release()
        print(f'Success! Video saved to {output_path}')

if __name__ == '__main__':

    args = parser.parse_args()

    vc = VideoConverter()
    vc.process(bag_path=args.bag_path, 
               topic=args.topic,
               output_path=args.output_path,
               fps=args.fps,
               width=args.width,
               height=args.height)