import cv2
import topic_if

class sim_cam_if(topic_if.SubscriberIf):
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
            wait(lambda: is_sim_cam_ready(self.sim_cam), timeout_seconds=120, waiting_for="Sim image")
            return self.sim_cam.read()
                
    def release(self):
        self.capture.release()