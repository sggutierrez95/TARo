from ultralytics import YOLO
 
# Load the model.
model = YOLO('yolov8n.pt')
 
# Training.
results = model.train(
   data='/home/azazool/ws_moveit/src/TARo/dataset/My First Project.v1i.yolov8/data.yaml',
   epochs=50,
   batch=8,
   device='cuda',
   deterministic=True,
   name='sim_waste')