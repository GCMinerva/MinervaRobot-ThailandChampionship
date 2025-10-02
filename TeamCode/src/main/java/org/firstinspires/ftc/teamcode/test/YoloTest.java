package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.tensorflow.lite.Interpreter;

import java.io.FileInputStream;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.MappedByteBuffer;
import java.nio.channels.FileChannel;
import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "YOLO Test", group = "Test")
public class YoloTest extends LinearOpMode {

    private OpenCvCamera camera;
    private YoloPipeline yoloPipeline;

    @Override
    public void runOpMode() {
        // Setup telemetry for both driver station and FTC dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        initCamera();

        telemetry.addData("Status", "Initialized");
        telemetry.addData(">", "Press START to begin");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                telemetryDetections();
                telemetry.update();

                // Control streaming with gamepad
                if (gamepad1.dpad_down) {
                    camera.pauseViewport();
                    telemetry.addData("Camera", "Paused");
                } else if (gamepad1.dpad_up) {
                    camera.resumeViewport();
                    telemetry.addData("Camera", "Running");
                }

                sleep(20);
            }
        }

        if (yoloPipeline != null) {
            yoloPipeline.release();
        }
        camera.stopStreaming();
    }

    private void initCamera() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        camera = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        yoloPipeline = new YoloPipeline();
        camera.setPipeline(yoloPipeline);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
                FtcDashboard.getInstance().startCameraStream(camera, 30);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Error", errorCode);
                telemetry.update();
            }
        });
    }

    private void telemetryDetections() {
        List<Detection> detections = yoloPipeline.getDetections();
        telemetry.addData("# Objects Detected", detections.size());

        // Display info for each detection
        for (Detection detection : detections) {
            telemetry.addLine()
                    .addData("Label", detection.label)
                    .addData("Confidence", String.format("%.2f", detection.confidence));
            telemetry.addLine()
                    .addData("Position", String.format("%.0f / %.0f", detection.x, detection.y))
                    .addData("Size", String.format("%.0f x %.0f", detection.width, detection.height));
        }
    }

    // Inner class for detection results
    static class Detection {
        String label;
        float confidence;
        float x, y, width, height;

        Detection(String label, float confidence, float x, float y, float width, float height) {
            this.label = label;
            this.confidence = confidence;
            this.x = x;
            this.y = y;
            this.width = width;
            this.height = height;
        }
    }

    // OpenCV Pipeline for YOLO inference
    class YoloPipeline extends OpenCvPipeline {
        private final List<Detection> detections = new ArrayList<>();
        private Interpreter interpreter;
        private int INPUT_SIZE = 320; // Will be updated based on model
        private final float CONFIDENCE_THRESHOLD = 0.25f;
        private final float IOU_THRESHOLD = 0.45f;
        private final String[] labels = {"purpleball", "greenball"};

        private ByteBuffer inputBuffer;
        private float[][][] outputBuffer; // Changed to 3D for [1, 7, 8400] or [1, 8400, 7]
        private int numBoxes = 8400;
        private int numCoords = 4 + labels.length; // 4 bbox coords + class scores

        private MappedByteBuffer loadModelFile(String modelPath) throws IOException {
            android.content.res.AssetFileDescriptor fileDescriptor =
                    hardwareMap.appContext.getAssets().openFd(modelPath);
            FileInputStream inputStream = new FileInputStream(fileDescriptor.getFileDescriptor());
            FileChannel fileChannel = inputStream.getChannel();
            long startOffset = fileDescriptor.getStartOffset();
            long declaredLength = fileDescriptor.getDeclaredLength();
            return fileChannel.map(FileChannel.MapMode.READ_ONLY, startOffset, declaredLength);
        }

        public YoloPipeline() {
            try {
                // Load TFLite model with options
                Interpreter.Options options = new Interpreter.Options();
                options.setNumThreads(4); // Use 4 threads for faster inference
                interpreter = new Interpreter(loadModelFile("yolo.tflite"), options);

                // Get input/output tensor shapes
                int[] inputShape = interpreter.getInputTensor(0).shape();
                int[] outputShape = interpreter.getOutputTensor(0).shape();

                // Update INPUT_SIZE based on model
                INPUT_SIZE = inputShape[1]; // Assuming [1, height, width, 3]

                telemetry.addData("Model Input Shape", String.format("[%d, %d, %d, %d]",
                        inputShape[0], inputShape[1], inputShape[2], inputShape[3]));
                telemetry.addData("Input Size", INPUT_SIZE);

                // Handle both possible output formats
                if (outputShape.length == 3) {
                    telemetry.addData("Model Output Shape", String.format("[%d, %d, %d]",
                            outputShape[0], outputShape[1], outputShape[2]));

                    // Determine the format: [1, 7, 8400] or [1, 8400, 7]
                    if (outputShape[1] == 6 || outputShape[1] == (4 + labels.length)) {
                        numCoords = outputShape[1];
                        numBoxes = outputShape[2];
                    } else {
                        numBoxes = outputShape[1];
                        numCoords = outputShape[2];
                    }
                    outputBuffer = new float[outputShape[0]][outputShape[1]][outputShape[2]];
                } else {
                    telemetry.addData("Model Output Shape", String.format("[%d, %d]",
                            outputShape[0], outputShape[1]));
                    // Fallback for 2D output
                    outputBuffer = new float[1][outputShape[1]][1];
                }

                telemetry.addData("Parsed", String.format("boxes=%d, coords=%d", numBoxes, numCoords));
                telemetry.update();

                // Allocate input buffer for INT8 quantized model with correct size
                int bufferSize = 1 * INPUT_SIZE * INPUT_SIZE * 3;
                inputBuffer = ByteBuffer.allocateDirect(bufferSize);
                inputBuffer.order(ByteOrder.nativeOrder());

                telemetry.addData("Buffer Size", String.format("%d bytes", bufferSize));
                telemetry.update();

            } catch (IOException e) {
                telemetry.addData("Model Load Error", e.getMessage());
                telemetry.update();
            }
        }

        @Override
        public Mat processFrame(Mat input) {
            if (interpreter == null) return input;

            try {
                // Preprocess image
                Mat resized = new Mat();
                Imgproc.resize(input, resized, new Size(INPUT_SIZE, INPUT_SIZE));

                // Convert to RGB
                Mat rgb = new Mat();
                Imgproc.cvtColor(resized, rgb, Imgproc.COLOR_RGBA2RGB);

                // Fill input buffer with INT8 values
                // For INT8 quantized models, we need to convert [0, 255] directly to bytes
                inputBuffer.rewind();
                byte[] pixelData = new byte[(int) (rgb.total() * rgb.channels())];
                rgb.get(0, 0, pixelData);
                inputBuffer.put(pixelData);

                rgb.release();
                resized.release();

                // Run inference
                interpreter.run(inputBuffer, outputBuffer);

                // Parse detections
                synchronized (detections) {
                    detections.clear();
                    parseDetections(input.width(), input.height());
                }

                // Draw bounding boxes
                drawDetections(input);

            } catch (Exception e) {
                telemetry.addData("Inference Error", e.getMessage());
            }

            return input;
        }

        private void parseDetections(int originalWidth, int originalHeight) {
            // YOLOv11 output format: [1, 7, 8400] where 7 = 4 bbox + 3 classes
            // or transposed: [1, 8400, 7]

            List<Detection> candidateDetections = new ArrayList<>();
            int[] outputShape = interpreter.getOutputTensor(0).shape();

            // Determine if output is [1, 7, 8400] or [1, 8400, 7]
            boolean isTransposed = (outputShape[1] == numBoxes);

            int numClasses = labels.length;

            // Parse each detection box
            for (int i = 0; i < numBoxes; i++) {
                float centerX, centerY, width, height;
                float[] classScores = new float[numClasses];

                if (isTransposed) {
                    // Format: [1, 8400, 7] - boxes are in dimension 1
                    centerX = outputBuffer[0][i][0];
                    centerY = outputBuffer[0][i][1];
                    width = outputBuffer[0][i][2];
                    height = outputBuffer[0][i][3];
                    for (int j = 0; j < numClasses; j++) {
                        classScores[j] = outputBuffer[0][i][4 + j];
                    }
                } else {
                    // Format: [1, 7, 8400] - boxes are in dimension 2
                    centerX = outputBuffer[0][0][i];
                    centerY = outputBuffer[0][1][i];
                    width = outputBuffer[0][2][i];
                    height = outputBuffer[0][3][i];
                    for (int j = 0; j < numClasses; j++) {
                        classScores[j] = outputBuffer[0][4 + j][i];
                    }
                }

                // Find max class confidence
                int maxClassIdx = 0;
                float maxConfidence = classScores[0];
                for (int j = 1; j < numClasses; j++) {
                    if (classScores[j] > maxConfidence) {
                        maxConfidence = classScores[j];
                        maxClassIdx = j;
                    }
                }

                // Filter by confidence threshold
                if (maxConfidence < CONFIDENCE_THRESHOLD) continue;

                // Scale to original image size
                float scaleX = (float) originalWidth / INPUT_SIZE;
                float scaleY = (float) originalHeight / INPUT_SIZE;

                // Convert from center format to corner format
                float x = (centerX - width / 2) * scaleX;
                float y = (centerY - height / 2) * scaleY;
                float w = width * scaleX;
                float h = height * scaleY;

                // Ensure valid bounding box
                if (x < 0 || y < 0 || w <= 0 || h <= 0) continue;

                String label = maxClassIdx < labels.length ? labels[maxClassIdx] : "unknown";
                candidateDetections.add(new Detection(label, maxConfidence, x, y, w, h));
            }

            // Apply Non-Maximum Suppression
            detections.addAll(nms(candidateDetections));
        }

        private List<Detection> nms(List<Detection> boxes) {
            if (boxes.isEmpty()) return new ArrayList<>();

            List<Detection> result = new ArrayList<>();
            boxes.sort((a, b) -> Float.compare(b.confidence, a.confidence));

            while (!boxes.isEmpty()) {
                Detection best = boxes.remove(0);
                result.add(best);

                boxes.removeIf(box -> iou(best, box) > IOU_THRESHOLD);
            }

            return result;
        }

        private float iou(Detection a, Detection b) {
            float x1 = Math.max(a.x, b.x);
            float y1 = Math.max(a.y, b.y);
            float x2 = Math.min(a.x + a.width, b.x + b.width);
            float y2 = Math.min(a.y + a.height, b.y + b.height);

            float intersection = Math.max(0, x2 - x1) * Math.max(0, y2 - y1);
            float areaA = a.width * a.height;
            float areaB = b.width * b.height;
            float union = areaA + areaB - intersection;

            return intersection / union;
        }

        private void drawDetections(Mat frame) {
            synchronized (detections) {
                for (Detection detection : detections) {
                    // Draw bounding box
                    Scalar color = new Scalar(0, 255, 0); // Default green
                    if (detection.label.equals("purpleball")) {
                        color = new Scalar(128, 0, 128); // Purple
                    } else if (detection.label.equals("greenball")) {
                        color = new Scalar(0, 255, 0); // Green
                    }

                    Point topLeft = new Point(detection.x, detection.y);
                    Point bottomRight = new Point(detection.x + detection.width, detection.y + detection.height);
                    Imgproc.rectangle(frame, topLeft, bottomRight, color, 2);

                    // Draw label
                    String labelText = String.format("%s: %.2f", detection.label, detection.confidence);
                    Imgproc.putText(frame, labelText,
                            new Point(detection.x, detection.y - 5),
                            Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, color, 2);
                }
            }
        }

        public List<Detection> getDetections() {
            synchronized (detections) {
                return new ArrayList<>(detections);
            }
        }

        public void release() {
            if (interpreter != null) {
                interpreter.close();
            }
        }
    }
}