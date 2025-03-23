from ultralytics import YOLO
import os
import cv2
import signal
import time
import sys
import argparse

import camera_interface as cam_if
import yolo_reader
import taro

# Arg setup
parser = argparse.ArgumentParser(description="A python-based robot program")

parser.add_argument("-id", type=int, required=True, help="Camera ID use to capture frames. Run print_cams.py for ref")
parser.add_argument("--imshow", action="store_true", help="Flag to display frames. Ctrl+C on terminal then SPACE while cam window active to safely stop program...")
args = parser.parse_args()

# RESOURCES
camera = cam_if.cam_if(args.id)

def signal_handler(sig, frame):
    camera.release()
    cv2.destroyAllWindows()
    print('Killed Program')
    sys.exit(0)

def find_weights_file(fn: str):
    root_folder = os.path.dirname(os.path.dirname(__file__))
    for root, dirs, files in os.walk(root_folder):
        if fn in files:
            return os.path.join(root, fn)


def main(frame : cv2.UMat, model: YOLO, camera: cam_if.cam_if):
    ret_frame = frame
    reader = yolo_reader.Yolo_Reader(model(frame))

    taro_robot.run(reader)

    dressed_frame = reader.get_dressed_frame(frame)
    return dressed_frame


if __name__ == "__main__":
    # Set up signal handler to safely release resources
    signal.signal(signal.SIGINT, signal_handler)

    # SYSTEM CONFIGURATION
    use_webcam =  True

    # RESOURCES
    camera = cam_if.cam_if(use_webcam)

    # initialize robot
    taro_robot = taro.TARo()

    # set up model
    weights_file = find_weights_file('best.pt')
    model = YOLO(weights_file) 

    while True:
        # rRead current frame
        ret, frame = camera.read()

        if ret:
            # Run a cycle of TARo Robot frame
            frame = main(frame, model, camera)

            # '--imshow' to see webcam frame
            if args.imshow:
                cv2.imshow('Webcam YOLO', frame)
                cv2.waitKey(0)
        else:
            raise ValueError("Cannot capture stream.")

