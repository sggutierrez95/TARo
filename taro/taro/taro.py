from enum import Enum
import math
import numpy as np
import cv2

import rclpy
from rclpy.executors import SingleThreadedExecutor

from spatialmath import SE3

from . import yolo_reader
import roboticstoolbox as rtb
from std_msgs.msg import Float64

class Robot_State_If():
    def __init__(self, robot : rtb.ERobot.URDF, arm_joints_if, gripper_joints_if, logger):
        self.robot = robot
        self.arm_joints = arm_joints_if
        self.gripper_joints = gripper_joints_if
        self.logger = logger
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
            self.logger.info('Publishing: "%f" arm-joint value' % msg.data)

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


class RobotState():
    def __init__(self):
        self.position = [0.0,0.0,0.0]
        self.joints = [0.0,0.0,0.0,0.0]



class TaroActions():
    def __init__(self, robot : rtb.ERobot, robot_if: Robot_State_If, logger):
        self.robot = robot
        self.robot_if = robot_if
        self.logger = logger

    def delta_x_move(self, delta_margin):
        # We will use a linear function to determine how far to move left or right
        m = 0.01
        b = 0.001
        return m*delta_margin + b

    def delta_base_joint_move(self, delta_margin):
        # We will use a linear function to determine how far to move left or right
        m = 0.0001
        b = math.radians(1)
        return m*delta_margin + b

    # def move_right(self, current_pos, delta_margin):
    #     new_pos = current_pos
    #     self.logger.info(f'Orig. Pos = [ {current_pos[0]}, {current_pos[1]}, {current_pos[2]} ] ')
    #     delta_x = self.delta_x_move(delta_margin)
    #     new_pos[1] = current_pos[1] + delta_x
    #     self.logger.info(f'New Pos = [ {new_pos[0]}, {new_pos[1]},{new_pos[2]} ] ')
    #     new_pos_se = SE3.Trans(new_pos)
    #     self.logger(f'SE Matrix {new_pos_se}')
    #     sol = self.robot.ikine_LM(new_pos_se, start=current_pos)
    #     if sol.success:
    #         self.logger.log('Solution found')
    #         self.robot_if.cmd_arm(sol.q.tolist())
    #         return new_pos
    #     else:
    #         self.logger.warn('Solution DNE!')
    #         raise ValueError('Cannot continue without a solution')

    def move_left(self, current_robot_state: RobotState, delta_margin):
        current_pos = current_robot_state.position
        self.logger.info(f'Orig. Pos = [ {current_pos[0]}, {current_pos[1]}, {current_pos[2]} ] ')
        delta_base_ang = self.delta_base_joint_move(delta_margin)
        current_robot_state.joints[0] = current_robot_state.joints[0] - delta_base_ang
        current_robot_state.position = self.robot_if.cmd_arm(current_robot_state.joints)
        current_pos = current_robot_state.position
        self.logger.info(f'New. Pos = [ {current_pos[0]}, {current_pos[1]}, {current_pos[2]} ] ')

    def move_right(self, current_robot_state: RobotState, delta_margin):
        current_pos = current_robot_state.position
        self.logger.info(f'Orig. Pos = [ {current_pos[0]}, {current_pos[1]}, {current_pos[2]} ] ')
        delta_base_ang = self.delta_base_joint_move(delta_margin)
        current_robot_state.joints[0] = current_robot_state.joints[0] + delta_base_ang
        current_robot_state.position = self.robot_if.cmd_arm(current_robot_state.joints)
        current_pos = current_robot_state.position
        self.logger.info(f'New. Pos = [ {current_pos[0]}, {current_pos[1]}, {current_pos[2]} ] ')

    def center_bb(self, current_robot_state: RobotState, reader : yolo_reader.Yolo_Reader):
        self.logger.info('Begining Bounding Box Centering')

        frame_results = reader.evaluate_frame()

        detection_to_center = None
        for detection in frame_results.detections:
            # we search the detections for any bounding boxes
            if len(detection.boxes) > 0:
                # We center the first one we see
                detection_to_center = detection
                box = detection.boxes[0]
                x_1, y_1, x_2, y_2 = map(int, box.xyxy[0])
                break

        # We need to see where the bounding box lies in the image
        orig_img_height, orig_img_width, channels = detection_to_center.orig_img.shape
        self.logger.info(f"Original Size of Img Height : { orig_img_height}, Width : {orig_img_width}")

        # We ned to get the optimal distance on each side of the bounding box
        # Get Bounding Box Height, Width
        bb_height = y_2 - y_1
        bb_width = x_2 - x_1

        optimal_width_margin = (orig_img_width - bb_width)/2
        optimal_height_margin = (orig_img_height - bb_height)/2
        self.logger.info(f"Centered Margins: height = { optimal_height_margin}, Width = {optimal_width_margin}")

        pixel_padding = 10
        padded_optimal_width_margin = optimal_width_margin + pixel_padding

        is_centered = True
        if x_1 > padded_optimal_width_margin:
            is_centered = False
            self.logger.info(f'BB too far to the right. Move Left X1 = {x_1} > ~{optimal_width_margin}')
            delta_margin = abs(x_1 - optimal_width_margin )
            self.move_left(current_robot_state, delta_margin)
        if (orig_img_width - x_2) > padded_optimal_width_margin:
            is_centered = False
            self.logger.info(f'BB too far to the Left. Move Right orig_img_width - x_2 = {orig_img_width - x_2} > ~{optimal_width_margin}')
            delta_margin = abs((orig_img_width - x_2) - optimal_width_margin )
            self.move_right(current_robot_state, delta_margin)
        
        # There is nothing to center
        return is_centered
        
        

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
        self.robot_state = RobotState()
        self.robot_state.position = self.robot.fkine([0.0,0.0,0.0,0.0]).t
        self.robot_state.joints = [0.0,0.0,0.0,0.0]
        self.robot_if = Robot_State_If(self.robot, arm_joint_if, gripper_joints_if, logger)
        self.actions = TaroActions(self.robot, self.robot_if, logger)
        self.logger = logger

    def run(self, reader : yolo_reader.Yolo_Reader, frame : cv2.Mat):
        match self.state:
            case Phases.INITIAL:
                self.robot_state.position = self.robot_if.cmd_arm(ARM_INITIAL_POS)
                self.robot_state.joints = ARM_INITIAL_POS
                self.state = Phases.DETECTION
            case Phases.DETECTION:
                # here is something where we want to develop a sophisticated searching algorithm until we found a detection
                frame_rets = reader.evaluate_frame()
                if frame_rets.waste_detected:
                    self.logger.info('Found waste!')
                    self.state = Phases.GRABBING
            case Phases.GRABBING:
                is_centered = self.actions.center_bb(self.robot_state, reader )
                if is_centered:
                    self.logger.info('Move straight')
                # self.state = Phases.DETECTION

    def destroy(self):
        self.robot_if.destroy()