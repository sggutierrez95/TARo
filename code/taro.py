from enum import Enum

import numpy as np

import yolo_reader
import cv2

# class syntax

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

    def run(self, reader : yolo_reader.Yolo_Reader):
        match self.state:
            case Phases.DETECTION:
                # here is something where we want to develop a sophisticated searching algorithm until we found a detection
                frame_rets = reader.evaluate_frame()
                if frame_rets.waste_detected:
                    print('Found waste!')
                    self.state = Phases.GRABBING
            case Phases.GRABBING:
                print('We are trying to grab')