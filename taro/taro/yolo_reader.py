import cv2

class FrameResults():
    def __init__(self):
        self.waste_detected = False

class Yolo_Reader():
    def __init__(self, results : list):
        self.results = results

    def get_dressed_frame(self, frame, logger):
        dressed_frame = frame
        for result in self.results:
            logger.info('dressing frame')
            boxes = result.boxes
            num_boxes = len(boxes)
            logger.info(f'Step 0 {num_boxes}')
            for box in boxes:
                logger.info('Step 1')
                num_boxes = num_boxes + 1
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                logger.info('Step 2')
                cv2.rectangle(dressed_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                logger.info('Step 3')
                class_id = int(box.cls[0])
                confidence = float(box.conf[0])
                logger.info('Step 4')
                label = f"{result.names[class_id]} {confidence:.2f}"
                logger.info(f'Class: {num_boxes}')
                cv2.putText(dressed_frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                

        return dressed_frame

    def evaluate_frame(self) -> FrameResults:
        frame_rets = FrameResults()

        frame_rets.waste_detected = len(self.results) > 0 and len(self.results[0].boxes) > 0

        return frame_rets
