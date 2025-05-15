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
    INITIAL   = 0
    DETECTION = 1
    GRABBING  = 2
    SORT      = 3
    DROP      = 4


class RobotState():
    def __init__(self):
        self.position = [0.0,0.0,0.0]
        self.joints = [0.0,0.0,0.0,0.0]



class TaroActions():
    def __init__(self, robot : rtb.ERobot, robot_if: Robot_State_If, logger):
        self.robot = robot
        self.robot_if = robot_if
        self.logger = logger
        self.prev_delta_margin = 0

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

    def delta_hand_joint_move(self, delta_margin):
        # We will use a linear function to determine how far to move left or right
        m = 0.001
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

    def move_up(self, current_robot_state: RobotState, delta_margin):
        current_pos = current_robot_state.position
        self.logger.info(f'Orig. Pos = [ {current_pos[0]}, {current_pos[1]}, {current_pos[2]} ] ')
        delta_base_ang = self.delta_base_joint_move(delta_margin)
        current_robot_state.joints[3] = current_robot_state.joints[3] + delta_base_ang
        current_robot_state.position = self.robot_if.cmd_arm(current_robot_state.joints)
        current_pos = current_robot_state.position
        self.logger.info(f'New. Pos = [ {current_pos[0]}, {current_pos[1]}, {current_pos[2]} ] ')

    def move_down(self, current_robot_state: RobotState, delta_margin):
        current_pos = current_robot_state.position
        self.logger.info(f'Orig. Pos = [ {current_pos[0]}, {current_pos[1]}, {current_pos[2]} ] ')
        delta_base_ang = self.delta_base_joint_move(delta_margin)
        current_robot_state.joints[3] = current_robot_state.joints[3] - delta_base_ang
        current_robot_state.position = self.robot_if.cmd_arm(current_robot_state.joints)
        current_pos = current_robot_state.position
        self.logger.info(f'New. Pos = [ {current_pos[0]}, {current_pos[1]}, {current_pos[2]} ] ')

    def delta_arm_joints(self, joints, delta_margin):

        # the way we move depends on different joint states
        joint_weight = [0.0,0.0,0.0,0.0]
        if joints[3] > math.radians(10):
            joint_weight = [0.0, 0.8, -0.5, 0.1]
        elif abs(joints[3]) <= math.radians(10):
            # if our wrist is semi-straight
            # change will need to coccur mostly in the lower joint
            joint_weight = [0.0, 0.4, -0.2, 0.0]
        elif joints[3] < -math.radians(10):
            joint_weight = [0.0, 0.9, 0.5, -0.1]
        else:
            self.logger.warn(f'Case not handled Joint 3: {joints[3]}')

        joint_delta = [0.0,0.0,0.0,0.0]

        # radians per pixel every 5 pixel difference will  move 2 degress
        m = math.radians(5) / 5
        delta_ang = m * delta_margin
        for idx, weight in enumerate(joint_weight):
            joint_delta[idx] = weight * delta_ang

        return joint_delta


    def move_straight(self, current_robot_state: RobotState, reader : yolo_reader.Yolo_Reader):
        self.logger.info('Moving Straight')
        frame_results = reader.evaluate_frame()

        if frame_results.detections is None:
            # we might have lost the detection in YOLO
            # so we might be ok to close so just assume  
            # we are close enough
            return True

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


        delta_joints = self.delta_arm_joints(current_robot_state.joints, x_1)
        sum_delta = 0.0
        for idx, delta in enumerate(delta_joints):
            sum_delta = sum_delta + delta
            current_robot_state.joints[idx] = current_robot_state.joints[idx] + delta
        current_robot_state.position = self.robot_if.cmd_arm(current_robot_state.joints)

        return sum_delta == 0.0 # ok to grab if we didn't have to move

    def center_bb(self, current_robot_state: RobotState, reader : yolo_reader.Yolo_Reader):
        self.logger.info('Begining Bounding Box Centering')

        frame_results = reader.evaluate_frame()

        if frame_results.detections is None:
            # we might have lost the detection in YOLO
            # so we might be ok to close so just assume its centered
            return True

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

        # We need to get the optimal distance on each side of the bounding box
        # Get Bounding Box Height, Width
        bb_height = y_2 - y_1
        bb_width = x_2 - x_1

        optimal_width_margin = (orig_img_width - bb_width)/2
        optimal_height_margin = (orig_img_height - bb_height)/2
        self.logger.info(f"Centered Margins: height = { optimal_height_margin}, Width = {optimal_width_margin}")

        pixel_padding = 10
        padded_optimal_width_margin = optimal_width_margin + pixel_padding
        padded_optimal_height_margin = optimal_height_margin + pixel_padding

        is_centered = True
        LOCKUP_THRESHOLD = 0 # if don't move then out config is messed up
        if x_1 > padded_optimal_width_margin:
            is_centered = False
            self.logger.info(f'BB too far to the right. Move Left X1 = {x_1} > ~{optimal_width_margin}')
            delta_margin = abs(x_1 - optimal_width_margin )
            if abs(delta_margin - self.prev_delta_margin) < LOCKUP_THRESHOLD:
                self.logger.info('Possible lock up. Moving back to inital.')
                override = Phases.INITIAL
                return None
            else:
                self.move_left(current_robot_state, delta_margin)
        if (orig_img_width - x_2) > padded_optimal_width_margin:
            is_centered = False
            self.logger.info(f'BB too far to the Left. Move Right orig_img_width - x_2 = {orig_img_width - x_2} > ~{optimal_width_margin}')
            delta_margin = abs((orig_img_width - x_2) - optimal_width_margin )
            if abs(delta_margin - self.prev_delta_margin) < LOCKUP_THRESHOLD:
                self.logger.info('Possible lock up. Moving back to inital.')
                override = Phases.INITIAL
                return None
            self.move_right(current_robot_state, delta_margin)

        if y_1 > padded_optimal_height_margin:
            is_centered = False
            self.logger.info(f'BB too low. Move up Y1 = {y_1} > ~{optimal_height_margin}')
            delta_margin = abs(y_1 - optimal_height_margin )
            if abs(delta_margin - self.prev_delta_margin) < LOCKUP_THRESHOLD:
                self.logger.info('Possible lock up. Moving back to inital.')
                override = Phases.INITIAL
                return None
            else:
                self.move_up(current_robot_state, delta_margin)
        if (orig_img_height - y_2) > padded_optimal_height_margin:
            is_centered = False
            self.logger.info(f'BB too high. Move down orig_img_height - y_2 = {orig_img_height - y_2} > ~{optimal_height_margin}')
            delta_margin = abs((orig_img_height - y_2) - optimal_height_margin )
            if abs(delta_margin - self.prev_delta_margin) < LOCKUP_THRESHOLD:
                self.logger.info('Possible lock up. Moving back to inital.')
                override = Phases.INITIAL
                return None
            else:
                self.move_down(current_robot_state, delta_margin)

        if not is_centered:
            self.prev_delta_margin = delta_margin
        # There is nothing to center
        return is_centered
        
DISCARDABLE = ['shoe']
COMPOSTABLE = ['banana']
RECYCLABLE  = ['water_bottle']
SALVAGABLE  = ['phone']

#                     mobile_base_arm_joint | base_forearm_joint | forearm_hand_joint |    hand_hand_2_joint
ARM_INITIAL_POS         = [            0.0,              0.8591,              0.8765,                1.3017]
ARM_BACK_LEFT_BIN_POS   = [        -2.8985,              0.2169,              1.0674,                1.2843]
ARM_BACK_RIGHT_BIN_POS  = [         2.9332,              0.2169,              0.9632,                1.3538]
ARM_FRONT_LEFT_BIN_POS  = [        -2.6902,                 0.0,              1.5707,                1.3624]
ARM_FRONT_RIGHT_BIN_POS = [         2.7597,                 0.0,               1.319,                1.5707]
#                              eef_eef_left_joint
GRIPPER_CLOSE           = [               math.radians(60)]       
GRIPPER_OPEN            = [                            0.0]       
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
        self.robot_if.cmd_gripper(GRIPPER_OPEN)

    def run(self, reader : yolo_reader.Yolo_Reader, frame : cv2.Mat):
        match self.state:
            case Phases.INITIAL:
                self.robot_if.cmd_gripper(GRIPPER_OPEN)
                self.robot_state.position = self.robot_if.cmd_arm(ARM_INITIAL_POS)
                self.robot_state.joints = ARM_INITIAL_POS
                self.state = Phases.DETECTION
                self.yolo_class = None
            case Phases.DETECTION:
                # here is something where we want to develop a sophisticated searching algorithm until we found a detection
                frame_rets = reader.evaluate_frame()
                if frame_rets.waste_detected:
                    self.yolo_class = reader.get_class()
                    self.logger.info('Found waste!')
                    self.state = Phases.GRABBING
            case Phases.GRABBING:
                is_centered = self.actions.center_bb(self.robot_state, reader )
                if is_centered is None:
                    self.robot_state.position = self.robot_if.cmd_arm(ARM_INITIAL_POS)
                    self.robot_state.joints = ARM_INITIAL_POS
                    self.state = Phases.INITIAL
                if is_centered:
                    ok_to_grab = self.actions.move_straight(self.robot_state, reader)
                    if ok_to_grab:
                        self.logger.info('Attempting a grab')
                        self.robot_if.cmd_gripper(GRIPPER_CLOSE)
                        self.state = Phases.SORT
            case Phases.SORT:
                if self.yolo_class in DISCARDABLE:
                    self.logger.info(f'Placing {self.yolo_class} in DISCARDABLE')
                    self.robot_state.position = self.robot_if.cmd_arm(ARM_BACK_LEFT_BIN_POS)
                    self.robot_state.joints = ARM_BACK_LEFT_BIN_POS
                elif self.yolo_class in SALVAGABLE:
                    self.logger.info(f'Placing {self.yolo_class} in SALVAGABLE')
                    self.robot_state.position = self.robot_if.cmd_arm(ARM_BACK_RIGHT_BIN_POS)
                    self.robot_state.joints = ARM_BACK_RIGHT_BIN_POS
                elif self.yolo_class in RECYCLABLE:
                    self.logger.info(f'Placing {self.yolo_class} in RECYCLABLE')
                    self.robot_state.position = self.robot_if.cmd_arm(ARM_FRONT_RIGHT_BIN_POS)
                    self.robot_state.joints = ARM_FRONT_RIGHT_BIN_POS
                else:
                    self.logger.info(f'Placing {self.yolo_class} in COMPOSTABLE')
                    self.robot_state.position = self.robot_if.cmd_arm(ARM_FRONT_LEFT_BIN_POS)
                    self.robot_state.joints = ARM_FRONT_LEFT_BIN_POS

                self.state = Phases.DROP
            case Phases.DROP:
                self.robot_if.cmd_gripper(GRIPPER_OPEN)
                self.state = Phases.INITIAL


    def destroy(self):
        self.robot_if.destroy()