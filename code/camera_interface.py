import cv2

class cam_if():
    def __init__(self, cam_id: int):
        self.capture = cv2.VideoCapture(cam_id)

        if not self.capture.isOpened():
            raise ValueError("Invalid cam id...try running ../code/util/print_cams.py ")

        self.capture.set(3, 507)
        self.capture.set(4, 380)

    def read(self):
        return self.capture.read() 

    def release(self):
        self.capture.release()