import cv2

class cam_if():
    def __init__(self, use_webcam: bool):
        if use_webcam:
            self.capture = cv2.VideoCapture(0)
        self.capture.set(3, 507)
        self.capture.set(4, 380)

    def read(self):
        return self.capture.read() 

    def release(self):
        self.capture.release()

    def display_frame(frame: cv2.UMat):
        cv2.imshow('Webcam YOLO', frame)
        cv2.waitKey(0)