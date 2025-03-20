from ultralytics import YOLO
 
# Load the model.
model = YOLO('yolov8n.pt')
 
# Training.
results = model.train(
   data='trash-sorting-v8.yaml',
   epochs=25,
   batch=8,
   device='cuda',
   deterministic=True,
   name='trash_sorter_custom')