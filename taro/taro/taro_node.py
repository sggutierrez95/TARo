from ultralytics import YOLO
import os
import cv2
import signal
import time
import sys
import argparse

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


from .interfaces.camera_interface import cam_if

from . import yolo_reader
from . import taro

# Arg setup
parser = argparse.ArgumentParser(description="A python-based robot program")

parser.add_argument("-id", type=int, required=True, help="Camera ID use to capture frames. Run print_cams.py for ref")
parser.add_argument("--imshow", action="store_true", help="Flag to display frames. Ctrl+C on terminal then SPACE while cam window active to safely stop program...")
# parser.add_argument("--ros-args", action="store_true")
args, UNKNOWN = parser.parse_known_args()

def signal_handler(sig, frame):
    camera.release()
    rclpy.shutdown()
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

class Taro_Node(topic_if.SubscriberIf):
    def __init__(self):
        super().__init__('taro_node', 'camera/image_raw', Image, self.main_callback)
        self.img = None
        self.new_img = False
        self.bridge = CvBridge()

        weights_file = find_weights_file('best.pt')
        self.yolo_model = YOLO(weights_file) 
        self.taro_robot = taro.TARo()

    def process_image_raw(self, data):
        self.get_logger().info('Received image')
        return cv2.resize(
            self.bridge.imgmsg_to_cv2(data, "bgr8"), (380, 507)
        )

    def destroy(self):
        self.taro_robot.destroy()
    
    def spin(self):
        self.taro.spin()

    def main_callback(self, data):
        try:
            # Process Raw Image to frame
            frame = self.process_image_raw(data)
            # YOLO the frame
            reader =  yolo_reader.Yolo_Reader(self.yolo_model(frame))
            # Run Taro
            taro_robot.run(reader)

            dressed_frame = reader.get_dressed_frame(frame)
            if args.imshow:
                cv2.imshow('Webcam YOLO', frame)
                cv2.waitKey(0)

        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")
    

def main(args=None):
    rclpy.init(args=args)
    taro_node = Taro_Node()

    # Spin Node
    taro_node.spin()
    rclpy.spin(taro_node)

    # Clean up
    taro_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    
    # Set up signal handler to safely release resources
    signal.signal(signal.SIGINT, signal_handler)
    main()