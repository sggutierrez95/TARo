from enum import Enum

import numpy as np
import cv2

import rclpy
from rclpy.executors import SingleThreadedExecutor

from . import yolo_reader
import roboticstoolbox as rtb
from std_msgs.msg import Float64

class Robot_State_If():
    def __init__(self, robot : rtb.ERobot.URDF, arm_joints_if, gripper_joints_if):
        self.robot = robot
        self.arm_joints = arm_joints_if
        self.gripper_joints = gripper_joints_if
        # for joint_publisher in self.arm_joints + self.gripper_joints:
        #     executor.add_node(joint_publisher)

    def cmd_arm(self, joint_cmds : list[float]):
        for joint_idx, joint_cmd, in enumerate(joint_cmds):
            msg = Float64()
            msg.data = joint_cmd
            self.arm_joints[joint_idx].publish(msg)

        return self.robot.fkine(joint_cmds).t

    def cmd_gripper(self, joint_cmds : list[float]):
        for joint_idx, joint_cmd, in enumerate(joint_cmds):
            msg = Float64()
            msg.data = joint_cmd
            self.gripper_joints[joint_idx].publish(msg)
            # self.gripper_joints[joint_idx].get_logger().info('Publishing: "%f" arm-joint value' % msg.data)

    def destroy(self):
        for joint_publisher in self.arm_joints + self.gripper_joints:
            joint_publisher.destroy_node()


    # <group_state name="start_infront" group="taro_arm">
    #     <joint name="base_forearm_joint" value="0.8765"/>
    #     <joint name="forearm_hand_joint" value="0.998"/>
    #     <joint name="hand_hand_2_joint" value="1.3017"/>
    #     <joint name="mobile_base_arm_joint" value="-0"/>
    # </group_state>
    # <group_state name="start_ahead" group="taro_arm">
    #     <joint name="base_forearm_joint" value="0.8591"/>
    #     <joint name="forearm_hand_joint" value="0.8765"/>
    #     <joint name="hand_hand_2_joint" value="0.6508"/>
    #     <joint name="mobile_base_arm_joint" value="0"/>
    # </group_state>
    # <group_state name="open" group="hand">
    #     <joint name="eef_eef_left_joint" value="0"/>
    # </group_state>
    # <group_state name="close" group="hand">
    #     <joint name="eef_eef_left_joint" value="0.5503"/>
    # </group_state>
    # <group_state name="back_left_bin" group="taro_arm">
    #     <joint name="base_forearm_joint" value="0.2169"/>
    #     <joint name="forearm_hand_joint" value="1.0674"/>
    #     <joint name="hand_hand_2_joint" value="1.2843"/>
    #     <joint name="mobile_base_arm_joint" value="-2.8985"/>
    # </group_state>
    # <group_state name="back_right_bin" group="taro_arm">
    #     <joint name="base_forearm_joint" value="0.2169"/>
    #     <joint name="forearm_hand_joint" value="0.9632"/>
    #     <joint name="hand_hand_2_joint" value="1.3538"/>
    #     <joint name="mobile_base_arm_joint" value="2.9332"/>
    # </group_state>
    # <group_state name="front_left_bin" group="taro_arm">
    #     <joint name="base_forearm_joint" value="0"/>
    #     <joint name="forearm_hand_joint" value="1.5707"/>
    #     <joint name="hand_hand_2_joint" value="1.3624"/>
    #     <joint name="mobile_base_arm_joint" value="-2.6902"/>
    # </group_state>
    # <group_state name="front_right_bin" group="taro_arm">
    #     <joint name="base_forearm_joint" value="0"/>
    #     <joint name="forearm_hand_joint" value="1.319"/>
    #     <joint name="hand_hand_2_joint" value="1.5707"/>
    #     <joint name="mobile_base_arm_joint" value="2.7597"/>
    # </group_state>

class Phases(Enum):
    INITIAL = 0
    DETECTION = 1
    GRABBING = 2

#                     mobile_base_arm_joint | base_forearm_joint | forearm_hand_joint |    hand_hand_2_joint
ARM_INITIAL_POS         = [            0.0,              0.8591,              0.8765,                1.3017]
ARM_BACK_LEFT_BIN_POS   = [        -2.8985,              0.2169,              1.0674,                1.2843]
ARM_BACK_RIGHT_BIN_POS  = [         2.9332,              0.2169,              0.9632,                1.3538]
ARM_FRONT_LEFT_BIN_POS  = [        -2.6902,                 0.0,              1.5707,                1.3624]
ARM_FRONT_RIGHT_BIN_POS = [         2.7597,                 0.0,               1.319,                1.5707]
class TARo():
    def __init__(self, urdf_path : str, arm_joint_if, gripper_joints_if, logger ):
        self.robot = rtb.ERobot.URDF(urdf_path , gripper='eef_link')
        self.state = Phases.INITIAL
        self.position = self.robot.fkine([0.0,0.0,0.0,0.0]).t
        self.robot_if = Robot_State_If(self.robot, arm_joint_if, gripper_joints_if)
        self.logger = logger

    def run(self, reader : yolo_reader.Yolo_Reader):
        match self.state:
            case Phases.INITIAL:
                self.position = self.robot_if.cmd_arm(ARM_INITIAL_POS)
                self.state = Phases.DETECTION
            case Phases.DETECTION:
                # here is something where we want to develop a sophisticated searching algorithm until we found a detection
                frame_rets = reader.evaluate_frame()
                if frame_rets.waste_detected:
                    self.logger.info('Found waste!')
                    self.state = Phases.GRABBING
            case Phases.GRABBING:
                self.logger.info('Trying to grab')
                self.state = Phases.DETECTION

    def destroy(self):
        self.robot_if.destroy()