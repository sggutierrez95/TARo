import cv2

class Yolo_Reader():
    def __init__(self, results):
        self.results = results

    def get_dressed_frame(self, frame):
        dressed_frame = frame
        for r in self.results:
            boxes = r.boxes
            for box in boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cv2.rectangle(dressed_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                class_id = int(box.cls[0])
                confidence = float(box.conf[0])
                label = f"{results.names[class_id]} {confidence:.2f}"
                cv2.putText(dressed_frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        return dressed_frame