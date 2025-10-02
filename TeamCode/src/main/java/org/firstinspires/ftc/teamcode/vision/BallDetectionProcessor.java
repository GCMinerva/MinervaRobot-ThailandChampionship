package org.firstinspires.ftc.teamcode.vision;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.RectF;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Mat;
import org.tensorflow.lite.Interpreter;
import org.tensorflow.lite.support.common.FileUtil;

import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.ArrayList;
import java.util.List;

public class BallDetectionProcessor implements VisionProcessor {

    // Model configuration
    private static final String MODEL_FILE = "model.tflite";
    private static final int INPUT_SIZE = 1280;  // Model expects 1280x1280 input
    private static final int NUM_DETECTIONS = 10;
    private static final float CONFIDENCE_THRESHOLD = 0.5f;

    // Class labels (index 0 = purpleball, index 1 = greenball)
    private static final String[] LABELS = {"purpleball", "greenball"};

    // TensorFlow Lite
    private Interpreter interpreter;
    private ByteBuffer inputBuffer;

    // Detection results
    private List<Detection> detections = new ArrayList<>();
    private final Object detectionsLock = new Object();

    // Paint for drawing
    private Paint purplePaint;
    private Paint greenPaint;
    private Paint textPaint;

    // Error tracking
    private String lastError = "";

    // Debug info
    private int rawDetectionCount = 0;
    private float maxConfidence = 0.0f;

    public static class Detection {
        public String label;
        public float confidence;
        public RectF boundingBox;

        public Detection(String label, float confidence, RectF boundingBox) {
            this.label = label;
            this.confidence = confidence;
            this.boundingBox = boundingBox;
        }
    }

    public BallDetectionProcessor() {
        initPaints();
    }

    private void initPaints() {
        purplePaint = new Paint();
        purplePaint.setColor(Color.rgb(128, 0, 128));
        purplePaint.setStyle(Paint.Style.STROKE);
        purplePaint.setStrokeWidth(4.0f);

        greenPaint = new Paint();
        greenPaint.setColor(Color.GREEN);
        greenPaint.setStyle(Paint.Style.STROKE);
        greenPaint.setStrokeWidth(4.0f);

        textPaint = new Paint();
        textPaint.setColor(Color.WHITE);
        textPaint.setTextSize(30);
        textPaint.setAntiAlias(true);
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        try {
            // Load model from assets using AppUtil context
            interpreter = new Interpreter(
                FileUtil.loadMappedFile(
                    org.firstinspires.ftc.robotcore.internal.system.AppUtil.getDefContext(),
                    MODEL_FILE
                )
            );

            // Initialize input buffer (1 * 320 * 320 * 3 uint8 values)
            inputBuffer = ByteBuffer.allocateDirect(1 * INPUT_SIZE * INPUT_SIZE * 3);
            inputBuffer.order(ByteOrder.nativeOrder());

            lastError = "";
        } catch (IOException e) {
            lastError = "Failed to load model: " + e.getMessage();
            throw new RuntimeException("Failed to load TensorFlow Lite model", e);
        }
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        if (interpreter == null) {
            return null;
        }

        try {
            // Prepare input buffer
            inputBuffer.rewind();

            // Convert Mat to ByteBuffer (resize to 320x320)
            Mat resized = new Mat();
            org.opencv.imgproc.Imgproc.resize(frame, resized,
                new org.opencv.core.Size(INPUT_SIZE, INPUT_SIZE));

            // Convert to RGB byte array
            byte[] frameData = new byte[INPUT_SIZE * INPUT_SIZE * 3];
            resized.get(0, 0, frameData);

            // Fill input buffer with uint8 values
            for (byte pixel : frameData) {
                inputBuffer.put(pixel);
            }

            resized.release();

            // Prepare output tensor - YOLO-style format [1][6][8400]
            // 8400 detections, each with 6 values (x, y, w, h, class0_conf, class1_conf)
            float[][][] outputData = new float[1][6][8400];

            // Run inference
            interpreter.run(inputBuffer, outputData);

            // Process results
            List<Detection> newDetections = new ArrayList<>();
            int frameWidth = frame.width();
            int frameHeight = frame.height();

            // Reset debug counters
            rawDetectionCount = 0;
            maxConfidence = 0.0f;

            // Process all 8400 detections
            for (int i = 0; i < 8400; i++) {
                // Get bounding box center and size (normalized 0-1)
                float centerX = outputData[0][0][i];
                float centerY = outputData[0][1][i];
                float width = outputData[0][2][i];
                float height = outputData[0][3][i];

                // Get class confidences (might need sigmoid activation)
                float conf0 = sigmoid(outputData[0][4][i]);  // purpleball confidence
                float conf1 = sigmoid(outputData[0][5][i]);  // greenball confidence

                // Determine class and confidence
                int classId;
                float confidence;
                if (conf0 > conf1) {
                    classId = 0;
                    confidence = conf0;
                } else {
                    classId = 1;
                    confidence = conf1;
                }

                // Track max confidence for debugging
                if (confidence > maxConfidence) {
                    maxConfidence = confidence;
                }

                // Count raw detections above a very low threshold for debugging
                if (confidence > 0.01f) {
                    rawDetectionCount++;
                }

                // Only keep detections above threshold
                if (confidence >= CONFIDENCE_THRESHOLD) {
                    // Convert center/size to corner coordinates (already normalized 0-1)
                    float left = (centerX - width / 2.0f);
                    float top = (centerY - height / 2.0f);
                    float right = (centerX + width / 2.0f);
                    float bottom = (centerY + height / 2.0f);

                    // Clamp to valid range
                    left = Math.max(0, Math.min(1, left));
                    top = Math.max(0, Math.min(1, top));
                    right = Math.max(0, Math.min(1, right));
                    bottom = Math.max(0, Math.min(1, bottom));

                    // Scale to frame size
                    RectF bbox = new RectF(
                        left * frameWidth,
                        top * frameHeight,
                        right * frameWidth,
                        bottom * frameHeight
                    );

                    newDetections.add(new Detection(LABELS[classId], confidence, bbox));
                }
            }

            synchronized (detectionsLock) {
                detections = newDetections;
            }

            lastError = "";
        } catch (Exception e) {
            lastError = "Inference error: " + e.getMessage();
        }

        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                           float scaleBmpPxToCanvasPx, float scaleCanvasDensity,
                           Object userContext) {
        synchronized (detectionsLock) {
            for (Detection detection : detections) {
                Paint paint = detection.label.equals("purpleball") ? purplePaint : greenPaint;

                // Scale bounding box to canvas
                RectF scaledBox = new RectF(
                    detection.boundingBox.left * scaleBmpPxToCanvasPx,
                    detection.boundingBox.top * scaleBmpPxToCanvasPx,
                    detection.boundingBox.right * scaleBmpPxToCanvasPx,
                    detection.boundingBox.bottom * scaleBmpPxToCanvasPx
                );

                // Draw bounding box
                canvas.drawRect(scaledBox, paint);

                // Draw label and confidence
                String text = String.format("%s: %.2f", detection.label, detection.confidence);
                canvas.drawText(text, scaledBox.left, scaledBox.top - 5, textPaint);
            }
        }

        // Draw error message if any
        if (!lastError.isEmpty()) {
            Paint errorPaint = new Paint();
            errorPaint.setColor(Color.RED);
            errorPaint.setTextSize(25);
            canvas.drawText(lastError, 10, 30, errorPaint);
        }
    }

    /**
     * Get current detections
     */
    public List<Detection> getDetections() {
        synchronized (detectionsLock) {
            return new ArrayList<>(detections);
        }
    }

    /**
     * Get detections of a specific color
     */
    public List<Detection> getDetectionsByLabel(String label) {
        List<Detection> filtered = new ArrayList<>();
        synchronized (detectionsLock) {
            for (Detection detection : detections) {
                if (detection.label.equals(label)) {
                    filtered.add(detection);
                }
            }
        }
        return filtered;
    }

    /**
     * Get closest detection to center of frame
     */
    public Detection getClosestDetectionToCenter(int frameWidth, int frameHeight) {
        Detection closest = null;
        float minDistance = Float.MAX_VALUE;
        float centerX = frameWidth / 2.0f;
        float centerY = frameHeight / 2.0f;

        synchronized (detectionsLock) {
            for (Detection detection : detections) {
                float detectionCenterX = (detection.boundingBox.left + detection.boundingBox.right) / 2.0f;
                float detectionCenterY = (detection.boundingBox.top + detection.boundingBox.bottom) / 2.0f;

                float distance = (float) Math.sqrt(
                    Math.pow(detectionCenterX - centerX, 2) +
                    Math.pow(detectionCenterY - centerY, 2)
                );

                if (distance < minDistance) {
                    minDistance = distance;
                    closest = detection;
                }
            }
        }

        return closest;
    }

    /**
     * Check if specific ball color is detected
     */
    public boolean isBallDetected(String label) {
        synchronized (detectionsLock) {
            for (Detection detection : detections) {
                if (detection.label.equals(label)) {
                    return true;
                }
            }
        }
        return false;
    }

    /**
     * Get last error message (useful for debugging)
     */
    public String getLastError() {
        return lastError;
    }

    /**
     * Get debug info for troubleshooting
     */
    public String getDebugInfo() {
        return String.format("Raw detections (>0.01): %d | Max conf: %.3f",
            rawDetectionCount, maxConfidence);
    }

    /**
     * Sigmoid activation function
     */
    private float sigmoid(float x) {
        return (float) (1.0 / (1.0 + Math.exp(-x)));
    }
}
