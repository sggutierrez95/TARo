from enum import Enum

import numpy as np
import cv2

import rclpy

from . import yolo_reader


from .interfaces.robot_state import Robot_State_Subscriber
# class syntax

class Phases(Enum):
    INIIIAL = 0
    DETECTION = 0
    GRABBING = 1

class Gripper():
    def __init__(self):
        self.position = np.array([0.0,0.0,0.0])

    def grab(self):
        print('We need to hook up sim. I wanna grab something')
        return

class Robot_State_Subscriber(topic_if.SubscriberIf):
    def __init__(self):
        super().__init__(topic_name, self.state_listener_callback)
        self.current_state = None
    def state_listener_callback(self, msg):
        self.current_state = msg.state

class Robot_State_Publisher(topic_if.PublisherIf):
    def __init__(self):
        super().__init__('taro_cmd', topic_name, msg_type ):
        self.current_state = None

class TARo():
    def __init__(self):
        self.state = Phases.DETECTION
        self.base_position = np.array([0.0,0.0,0.0])
        self.robot_state = Robot_State_Subscriber()

    def run(self, reader : yolo_reader.Yolo_Reader):
        match self.state:
            case Phases.INITAL:
                print('go to intial')
            case Phases.DETECTION:
                # here is something where we want to develop a sophisticated searching algorithm until we found a detection
                frame_rets = reader.evaluate_frame()
                if frame_rets.waste_detected:
                    print('Found waste!')
                    self.state = Phases.GRABBING
            case Phases.GRABBING:
                print('We are trying to grab')
    def destroy():
        self.robot_state.destroy_node()
    def spin():
        rclpy.spin(self.robot_state)