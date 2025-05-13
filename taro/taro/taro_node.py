from ultralytics import YOLO
import os
import cv2
import signal
import time
import sys
import argparse
import atexit

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Float64


from . import yolo_reader
from . import taro
from .interfaces.topic_if import SubscriberIf

# Arg setup
parser = argparse.ArgumentParser(description="A python-based robot program")

parser.add_argument("-id", type=int, required=True, help="Camera ID use to capture frames. Run print_cams.py for ref")
parser.add_argument("--imshow", action="store_true", help="Flag to display frames. Ctrl+C on terminal then SPACE while cam window active to safely stop program...")
parser.add_argument("-urdf", type=str, required=True, help="Path to the urdf of the robot")
# parser.add_argument("--ros-args", action="store_true")
parsed_args, UNKNOWN = parser.parse_known_args()

def signal_handler(sig, frame):
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

class Taro_Node(SubscriberIf):
    def __init__(self, urdf_path : str ):
        super().__init__('taro_node', 'camera/image_raw', Image, self.main_callback)
        self.arm_joint_if = [
            self.create_publisher(Float64, '/joint0/cmd_pos',10),
            self.create_publisher(Float64, '/joint1/cmd_pos',10),
            self.create_publisher(Float64, '/joint2/cmd_pos',10),
            self.create_publisher(Float64, '/joint3/cmd_pos',10)
        ]
        self.gripper_joints_if = [
            self.create_publisher(Float64, '/joint4/cmd_pos',10)
        ]
        self.img = None
        self.new_img = False
        self.bridge = CvBridge()

        weights_file = find_weights_file('sim_best.pt')
        self.yolo_model = YOLO(weights_file)
        self.taro_robot = taro.TARo(urdf_path, self.arm_joint_if, self.gripper_joints_if, self.get_logger())

    def process_image_raw(self, data):
        # self.get_logger().info('Received image')
        # Convert to cv2 format
        raw_img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        # Crop image to avoid looking at arms
        # so crop the left and right sides
        x_start, x_end = 235, 435
        cropped_image = raw_img[:, x_start:x_end]
        return cropped_image

    def destroy(self):
        self.taro_robot.destroy()

    def main_callback(self, data):
        try:
            # Process Raw Image to frame
            frame = self.process_image_raw(data)
            # YOLO the frame
            reader =  yolo_reader.Yolo_Reader(self.yolo_model(frame))
            # Run Taro
            self.taro_robot.run(reader, frame)

            dressed_frame = reader.get_dressed_frame(frame, self.get_logger())
            if parsed_args.imshow:
                cv2.imshow('Webcam YOLO', dressed_frame)
                cv2.waitKey(0)

        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")
    

def main(args=None):
    # Set up signal handler to safely release resources
    signal.signal(signal.SIGINT, signal_handler)
    rclpy.init(args=args)

    taro_node = Taro_Node(parsed_args.urdf)

    # Spin Node
    rclpy.spin(taro_node)

    # Clean up
    def cleanup():
        taro_node.destroy()
        taro_node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

    atexit.register(cleanup)

if __name__ == '__main__':
    main()