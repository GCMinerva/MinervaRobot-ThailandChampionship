# Fix YOLOv11 Model Export Issue

## Problem
Your current model expects **1280×1280 input** (4,915,200 bytes) but the metadata says 640×640.
This causes the error: `cannot copy to a TensorFlowLite tensor with 4915200 bytes from a Java Buffer with 1228800 bytes`

## Solution: Re-export the model correctly

### Step 1: Navigate to where you trained your model
Find the folder with `best.pt` (likely in `model/` or your training directory)

### Step 2: Run this Python code

```python
from ultralytics import YOLO

# Load your trained model
model = YOLO('best.pt')

# Export with 320x320 for FAST real-time inference (RECOMMENDED FOR FTC)
model.export(
    format='tflite',
    imgsz=320,        # ← 320x320 is FAST and accurate for ball detection
    int8=True,
    nms=False,        # NMS done in Java
    simplify=True
)
```

### Step 3: Copy the exported model
The export creates `best_saved_model/best_int8.tflite`

Copy it to: `TeamCode/src/main/assets/yolo.tflite`

### Step 4: Verify
The new model should:
- Expect input: `[1, 320, 320, 3]` = 307,200 bytes ✓
- Run at 30+ FPS on robot ✓
- Accurate enough for purple/green ball detection ✓

## Why 320×320 is Best for FTC

| Size | Speed | Accuracy | Recommendation |
|------|-------|----------|----------------|
| 1280×1280 | ❌ Very slow (16× pixels) | ✓✓✓ Best | Not for real-time |
| 640×640 | ⚠️ Slow (4× pixels) | ✓✓ Good | OK for static vision |
| **320×320** | ✓✓✓ **FAST** | ✓ **Good enough** | **✓ BEST for FTC** |

For ball detection in FTC, 320×320 is perfect:
- ✓ Balls are large, easy-to-detect objects (not tiny)
- ✓ Speed matters MORE than accuracy for autonomous
- ✓ Lower latency = better robot control loop
- ✓ Can run at 30+ FPS on Control Hub

## Why this happened
Your model was exported at 1280×1280 instead of the intended size.

## Current Status
Your Java code will auto-detect whatever size you export.
**Smaller input = faster inference = better autonomous performance!**
