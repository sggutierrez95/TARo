from ultralytics import YOLO
import os
import cv2
import signal
import time
import sys

import camera_interface as cam_if
import yolo_reader

# CONFIGURATION
use_webcam =  True

# RESOURCES
camera = cam_if.cam_if(use_webcam)

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
    reader = yolo_reader.Yolo_Reader(model(frame))
    dressed_frame = reader.get_dressed_frame(frame)

    camera.display_frame(dressed_frame)


if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)

    weights_file = find_weights_file('best.pt')
    model = YOLO(weights_file) 

    while True:
        ret, frame = camera.read()
        if not ret:
            main(frame, model)

