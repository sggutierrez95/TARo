import cv2
from ultralytics.engine.results import Results as Yolo_Result
class FrameResults():
    def __init__(self):
        self.waste_detected = False
        self.detections = None

class Yolo_Reader():
    def __init__(self, results : list[Yolo_Result]):
        self.results = results

    def get_dressed_frame(self, frame, logger):
        dressed_frame = frame
        for result in self.results:
            boxes = result.boxes
            for box in boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cv2.rectangle(dressed_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                class_id = int(box.cls[0])
                confidence = float(box.conf[0])
                label = f"{result.names[class_id]} {confidence:.2f}"
                cv2.putText(dressed_frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                
        return dressed_frame

    def get_class(self):
        for result in self.results:
            boxes = result.boxes
            for box in boxes:
                class_id = int(box.cls[0])
                return result.names[class_id]

    def evaluate_frame(self) -> FrameResults:
        frame_rets = FrameResults()

        frame_rets.waste_detected = len(self.results) > 0 and len(self.results[0].boxes) > 0
        if frame_rets.waste_detected:
            frame_rets.detections = self.results

        return frame_rets
