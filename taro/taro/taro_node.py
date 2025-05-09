from ultralytics import YOLO
import os
import cv2
import signal
import time
import sys
import argparse

from .interfaces.camera_interface import cam_if

from . import yolo_reader
# import taro

# Arg setup
parser = argparse.ArgumentParser(description="A python-based robot program")

parser.add_argument("-id", type=int, required=True, help="Camera ID use to capture frames. Run print_cams.py for ref")
parser.add_argument("--imshow", action="store_true", help="Flag to display frames. Ctrl+C on terminal then SPACE while cam window active to safely stop program...")
# parser.add_argument("--ros-args", action="store_true")
args, UNKNOWN = parser.parse_known_args()


############### TOPIC_IF ###############
from rclpy.node import Node
from std_msgs.msg import String

class SubscriberIf(Node):

    def __init__(self, name, callback_handle: callable):
        super().__init__(name)
        self.subscription = self.create_subscription(
            String,
            'topic',
            callback_handle,
            10)
        self.subscription  # prevent unused variable warning


############### TOPIC_IF ###############


############### CAMERA_INTERFACE ###############
from waiting import wait
class sim_cam_if(SubscriberIf):
    def __init__(self, topic_name):
        super().__init__(topic_name, self.img_listener_callback)
        self.img = None
        self.new_img = False
    def img_listener_callback(self, msg):
        self.img = msg.data
        self.new_img = True

    def read(self):
        if self.new_img:
            return self.img
        else:
            raise ValueError('Wait until there is an img.')
    def release(self):
        self.destroy_node()

class cam_if():
    def __init__(self, cam_id: int):
        self.sim_cam = None
        if cam_id >= 0:
            self.capture = cv2.VideoCapture(cam_id)
            if not self.capture.isOpened():
                raise ValueError("Invalid cam id...try running ../code/util/print_cams.py ")
            self.capture.set(3, 507)
            self.capture.set(4, 380)
        else:
            self.sim_cam = sim_cam_if('/camera/image_raw')

    def is_sim_cam_ready(sim_cam : sim_cam_if):
        return sim_cam.new_img
        
    def read(self):
        if self.sim_cam is None:
            return self.capture.read() 
        else:
            wait(lambda: self.is_sim_cam_ready(self.sim_cam), timeout_seconds=120, waiting_for="Sim image")
            return self.sim_cam.read()
                
    def release(self):
        self.capture.release()
############### CAMERA_INTERFACE ###############


############### ROBOT_STATE ###############
class Robot_State_If(SubscriberIf):
    def __init__(self, topic_name):
        super().__init__(topic_name, self.state_listener_callback)
        self.state = None
    def state_listener_callback(self, msg):
        self.state = msg.state
############### ROBOT_STATE ###############


############### YOLO_READER ###############
class FrameResults():
    def __init__(self):
        self.waste_detected = False

class Yolo_Reader():
    def __init__(self, results : list):
        self.results = results

    def get_dressed_frame(self, frame):
        dressed_frame = frame
        for result in self.results:
            boxes = result.boxes
            for box in boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cv2.rectangle(dressed_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                class_id = int(box.cls[0])
                confidence = float(box.conf[0])
                label = f"{result.names[class_id]} {confidence:.2f}"
                cv2.putText(dressed_frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        return dressed_frame

    def evaluate_frame(self) -> FrameResults:
        frame_rets = FrameResults()

        frame_rets.waste_detected = self.results != []

        return frame_rets
############### YOLO_READER ###############

############### TARO ###############
from enum import Enum

import numpy as np

class Phases(Enum):
    DETECTION = 0
    GRABBING = 1

class Gripper():
    def __init__(self):
        self.position = np.array([0.0,0.0,0.0])

    def grab(self):
        print('We need to hook up sim. I wanna grab something')
        return

class TARo():
    def __init__(self):
        self.state = Phases.DETECTION
        self.base_position = np.array([0.0,0.0,0.0])

    def run(self, reader : Yolo_Reader):
        match self.state:
            case Phases.DETECTION:
                # here is something where we want to develop a sophisticated searching algorithm until we found a detection
                frame_rets = reader.evaluate_frame()
                if frame_rets.waste_detected:
                    print('Found waste!')
                    self.state = Phases.GRABBING
            case Phases.GRABBING:
                print('We are trying to grab')
############### TARO ###############


# RESOURCES
camera = cam_if(args.id)

def signal_handler(sig, frame):
    camera.release()
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


def taro_main(frame : cv2.UMat, model: YOLO, camera: cam_if, taro_robot : TARo):
    ret_frame = frame
    reader = Yolo_Reader(model(frame))

    taro_robot.run(reader)

    dressed_frame = reader.get_dressed_frame(frame)
    return dressed_frame


def main():
    # Set up signal handler to safely release resources
    signal.signal(signal.SIGINT, signal_handler)

    # initialize robot
    taro_robot = TARo()

    # set up model
    weights_file = find_weights_file('best.pt')
    model = YOLO(weights_file) 

    while True:
        # rRead current frame
        ret, frame = camera.read()

        if ret:
            # Run a cycle of TARo Robot frame
            frame = taro_main(frame, model, camera, taro_robot)

            # '--imshow' to see webcam frame
            if args.imshow:
                cv2.imshow('Webcam YOLO', frame)
                cv2.waitKey(0)
        else:
            raise ValueError("Cannot capture stream.")

