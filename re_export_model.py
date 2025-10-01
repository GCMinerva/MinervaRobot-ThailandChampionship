"""
Script to re-export YOLOv11 model with older opset for compatibility
Run this where you originally trained your model
"""

from ultralytics import YOLO

# Load your trained model
model = YOLO('best.pt')  # Use your actual .pt file path

# Export with older opset for better compatibility
model.export(
    format='tflite',
    imgsz=640,
    int8=True,
    opset=12,  # Use older opset for TFLite 2.8-2.11 compatibility
    nms=False,  # You're doing NMS in Java
    simplify=True  # Simplify model for better compatibility
)

print("Model exported successfully! Copy the new .tflite file to TeamCode/src/main/assets/")
