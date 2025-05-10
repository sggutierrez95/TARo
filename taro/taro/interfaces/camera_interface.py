import cv2
import os
from waiting import wait

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import rclpy

from . import topic_if


class sim_cam_if(topic_if.SubscriberIf):
    def __init__(self, topic_name):
        super().__init__('sim_cam', topic_name, Image, self.img_listener_callback)
        self.img = None
        self.new_img = False
        self.bridge = CvBridge()

    def img_listener_callback(self, data):
        try:
            self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")
            cv2.imshow("Received Image", self.img)
            cv2.waitKey(1)
            self.get_logger().info('Received image')
        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

    def read(self):
        return self.img

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
            self.sim_cam = sim_cam_if('camera/image_raw')
            rclpy.spin(self.sim_cam)

    def is_sim_cam_ready(self, sim_cam : sim_cam_if):
        return sim_cam.new_img
        
    def read(self):
        if self.sim_cam is None:
            return self.capture.read() 
        else:
            img = self.sim_cam.read()
            if img is not None:
                img = cv2.resize(img, (380, 570))
        #     wait(lambda: self.is_sim_cam_ready(self.sim_cam), timeout_seconds=120, waiting_for="Sim image")
        return True, img
                
    def release(self):
        self.capture.release()