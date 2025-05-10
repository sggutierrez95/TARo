from ultralytics import YOLO
import os
import cv2
import signal
import time
import sys
import argparse

import rclpy

from .interfaces.camera_interface import cam_if

from . import yolo_reader
from . import taro

# Arg setup
parser = argparse.ArgumentParser(description="A python-based robot program")

parser.add_argument("-id", type=int, required=True, help="Camera ID use to capture frames. Run print_cams.py for ref")
parser.add_argument("--imshow", action="store_true", help="Flag to display frames. Ctrl+C on terminal then SPACE while cam window active to safely stop program...")
# parser.add_argument("--ros-args", action="store_true")
args, UNKNOWN = parser.parse_known_args()

# RESOURCES
camera = cam_if(args.id)

def signal_handler(sig, frame):
    camera.release()
    rclpy.shutdown()
    cv2.destroyAllWindows()
    print('Killed Program')
    sys.exit(0)

def find_weights_file(fn: str):
    dir1 = os.path.dirname(os.path.dirname(__file__))
    dir2 = os.path.dirname(dir1)
    dir3 = os.path.dirname(dir2)

    for root_folder in [dir1, dir2, dir3]:
      for root, dirs, files in os.walk(root_folder):
        if fn in files:
            return os.path.join(root, fn)


def taro_main(frame : cv2.UMat, model: YOLO, camera: cam_if, taro_robot : taro.TARo):
    ret_frame = frame
    reader = yolo_reader.Yolo_Reader(model(frame))

    taro_robot.run(reader)

    dressed_frame = reader.get_dressed_frame(frame)
    return dressed_frame


def main():
    node = rclpy.create_node('taro_node')
    logger = node.get_logger()
    # Set up signal handler to safely release resources
    signal.signal(signal.SIGINT, signal_handler)

    # initialize robot
    taro_robot = taro.TARo()

    # set up model
    weights_file = find_weights_file('best.pt')
    model = YOLO(weights_file) 

    logger.warn('I am going to run')
    while True:
        # rRead current frame
        ret, frame = camera.read()
        logger.warn('I am running')
        if ret and frame is not None:
            logger.warn('I am processing an img...')
            # Run a cycle of TARo Robot frame
            frame = taro_main(frame, model, camera, taro_robot)

            # '--imshow' to see webcam frame
            if args.imshow:
                cv2.imshow('Webcam YOLO', frame)
                cv2.waitKey(0)
        else:
           logger.warn('I am waiting for an img...')

