from enum import Enum

import yolo_reader
import cv2

# class syntax

class States(Enum):
    SEARCH = 0
class TARo():
    def __init__(self):
        self.state = States.SEARCH

    def run(self, reader : yolo_reader.Yolo_Reader):
        match self.state:
            case SEARCH:
                frame_rets = reader.evaluate_frame()
                return