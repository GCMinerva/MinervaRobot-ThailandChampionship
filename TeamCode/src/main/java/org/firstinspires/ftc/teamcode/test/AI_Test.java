package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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
import java.nio.FloatBuffer;
import java.nio.MappedByteBuffer;
import java.nio.channels.FileChannel;
import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "AI Test", group = "Test")
public class AI_Test extends LinearOpMode {

    private OpenCvCamera camera;
    private AIPipeline aiPipeline;

    @Override
    public void runOpMode() {
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

        if (aiPipeline != null) {
            aiPipeline.release();
        }
        camera.stopStreaming();
    }

    private void initCamera() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        camera = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        aiPipeline = new AIPipeline();
        camera.setPipeline(aiPipeline);

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
        List<Detection> detections = aiPipeline.getDetections();
        telemetry.addData("# Objects Detected", detections.size());

        for (Detection detection : detections) {
            telemetry.addLine()
                    .addData("Label", detection.label)
                    .addData("Confidence", String.format("%.2f", detection.confidence));
            telemetry.addLine()
                    .addData("Position", String.format("%.0f / %.0f", detection.x, detection.y))
                    .addData("Size", String.format("%.0f x %.0f", detection.width, detection.height));
        }
    }

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

    class AIPipeline extends OpenCvPipeline {
        private final List<Detection> detections = new ArrayList<>();
        private Interpreter interpreter;
        private int inputWidth = 320;
        private int inputHeight = 320;
        private int inputChannels = 3;
        private Interpreter.DataType inputDataType = Interpreter.DataType.UINT8;
        private float inputScale = 1.0f;
        private int inputZeroPoint = 0;
        private final float CONFIDENCE_THRESHOLD = 0.25f;
        private final float IOU_THRESHOLD = 0.45f;
        private final String[] labels = {"purpleball", "greenball"};

        private ByteBuffer inputBuffer;
        private float[][][] outputBuffer;
        private Object rawOutputBuffer;
        private byte[][][] quantizedOutputBuffer;
        private Interpreter.DataType outputDataType = Interpreter.DataType.FLOAT32;
        private float outputScale = 1.0f;
        private int outputZeroPoint = 0;
        private int numBoxes = 8400;
        private int numCoords = 4 + labels.length;

        private MappedByteBuffer loadModelFile(String modelPath) throws IOException {
            android.content.res.AssetFileDescriptor fileDescriptor =
                    hardwareMap.appContext.getAssets().openFd(modelPath);
            FileInputStream inputStream = new FileInputStream(fileDescriptor.getFileDescriptor());
            FileChannel fileChannel = inputStream.getChannel();
            long startOffset = fileDescriptor.getStartOffset();
            long declaredLength = fileDescriptor.getDeclaredLength();
            return fileChannel.map(FileChannel.MapMode.READ_ONLY, startOffset, declaredLength);
        }

        AIPipeline() {
            try {
                Interpreter.Options options = new Interpreter.Options();
                options.setNumThreads(4);
                interpreter = new Interpreter(loadModelFile("yolo.tflite"), options);

                int[] inputShape = interpreter.getInputTensor(0).shape();
                int[] outputShape = interpreter.getOutputTensor(0).shape();

                inputHeight = inputShape.length > 1 ? inputShape[1] : inputHeight;
                inputWidth = inputShape.length > 2 ? inputShape[2] : inputWidth;
                inputChannels = inputShape.length > 3 ? inputShape[3] : inputChannels;

                inputDataType = interpreter.getInputTensor(0).dataType();
                Interpreter.QuantizationParams inputQuant = interpreter.getInputTensor(0).quantizationParams();
                inputScale = inputQuant.getScale() == 0 ? 1.0f : inputQuant.getScale();
                inputZeroPoint = inputQuant.getZeroPoint();

                telemetry.addData("Model Input Shape", String.format("[%d, %d, %d, %d]",
                        inputShape[0], inputShape[1], inputShape[2], inputShape[3]));
                telemetry.addData("Input Size", String.format("%dx%dx%d", inputWidth, inputHeight, inputChannels));
                telemetry.addData("Input Type", inputDataType);

                if (outputShape.length == 3) {
                    telemetry.addData("Model Output Shape", String.format("[%d, %d, %d]",
                            outputShape[0], outputShape[1], outputShape[2]));

                    if (outputShape[1] == 6 || outputShape[1] == (4 + labels.length)) {
                        numCoords = outputShape[1];
                        numBoxes = outputShape[2];
                    } else {
                        numBoxes = outputShape[1];
                        numCoords = outputShape[2];
                    }
                    allocateOutputBuffer(outputShape);
                } else {
                    telemetry.addData("Model Output Shape", String.format("[%d, %d]",
                            outputShape[0], outputShape[1]));
                    allocateOutputBuffer(new int[]{1, outputShape[0], outputShape[1]});
                }

                telemetry.addData("Parsed", String.format("boxes=%d, coords=%d", numBoxes, numCoords));

                int bufferSize = interpreter.getInputTensor(0).numBytes();
                inputBuffer = ByteBuffer.allocateDirect(bufferSize);
                inputBuffer.order(ByteOrder.nativeOrder());

                telemetry.addData("Input Buffer Size", String.format("%d bytes", bufferSize));

                outputDataType = interpreter.getOutputTensor(0).dataType();
                Interpreter.QuantizationParams outputQuant = interpreter.getOutputTensor(0).quantizationParams();
                outputScale = outputQuant.getScale() == 0 ? 1.0f : outputQuant.getScale();
                outputZeroPoint = outputQuant.getZeroPoint();
                telemetry.addData("Output Type", outputDataType);
                telemetry.update();

            } catch (IOException e) {
                telemetry.addData("Model Load Error", e.getMessage());
                telemetry.update();
            }
        }

        private void allocateOutputBuffer(int[] outputShape) {
            outputBuffer = new float[outputShape[0]][outputShape[1]][outputShape[2]];
            Interpreter.DataType type = interpreter.getOutputTensor(0).dataType();
            if (type == Interpreter.DataType.FLOAT32) {
                rawOutputBuffer = outputBuffer;
                quantizedOutputBuffer = null;
            } else {
                quantizedOutputBuffer = new byte[outputShape[0]][outputShape[1]][outputShape[2]];
                rawOutputBuffer = quantizedOutputBuffer;
            }
        }

        @Override
        public Mat processFrame(Mat input) {
            if (interpreter == null) {
                return input;
            }

            try {
                Mat resized = new Mat();
                Imgproc.resize(input, resized, new Size(inputWidth, inputHeight));

                Mat processed = prepareInputChannels(resized);

                fillInputBuffer(processed);

                processed.release();
                resized.release();

                inputBuffer.rewind();
                interpreter.run(inputBuffer, rawOutputBuffer);

                if (outputDataType != Interpreter.DataType.FLOAT32 && quantizedOutputBuffer != null) {
                    dequantizeOutput();
                }

                synchronized (detections) {
                    detections.clear();
                    parseDetections(input.width(), input.height());
                }

                drawDetections(input);

            } catch (Exception e) {
                telemetry.addData("Inference Error", e.getMessage());
            }

            return input;
        }

        private Mat prepareInputChannels(Mat resized) {
            Mat converted = new Mat();
            if (inputChannels == 3) {
                Imgproc.cvtColor(resized, converted, Imgproc.COLOR_RGBA2RGB);
            } else if (inputChannels == 1) {
                Imgproc.cvtColor(resized, converted, Imgproc.COLOR_RGBA2GRAY);
            } else if (inputChannels == 4) {
                resized.copyTo(converted);
            } else {
                Imgproc.cvtColor(resized, converted, Imgproc.COLOR_RGBA2RGB);
            }
            return converted;
        }

        private void fillInputBuffer(Mat processed) {
            inputBuffer.rewind();

            int totalPixels = (int) (processed.total() * processed.channels());
            byte[] pixelData = new byte[totalPixels];
            processed.get(0, 0, pixelData);

            if (inputDataType == Interpreter.DataType.FLOAT32) {
                FloatBuffer floatBuffer = inputBuffer.asFloatBuffer();
                floatBuffer.position(0);
                for (int i = 0; i < totalPixels; i++) {
                    float normalized = (pixelData[i] & 0xFF) / 255.0f;
                    floatBuffer.put(normalized);
                }
            } else if (inputDataType == Interpreter.DataType.UINT8) {
                inputBuffer.put(pixelData);
            } else if (inputDataType == Interpreter.DataType.INT8) {
                for (int i = 0; i < totalPixels; i++) {
                    float realValue = (pixelData[i] & 0xFF) / 255.0f;
                    int quantized = Math.round(realValue / inputScale + inputZeroPoint);
                    quantized = Math.max(-128, Math.min(127, quantized));
                    inputBuffer.put((byte) quantized);
                }
            } else {
                inputBuffer.put(pixelData);
            }
        }

        private void dequantizeOutput() {
            if (quantizedOutputBuffer == null) {
                return;
            }
            for (int i = 0; i < quantizedOutputBuffer.length; i++) {
                for (int j = 0; j < quantizedOutputBuffer[i].length; j++) {
                    for (int k = 0; k < quantizedOutputBuffer[i][j].length; k++) {
                        int quantized = quantizedOutputBuffer[i][j][k] & 0xFF;
                        if (outputDataType == Interpreter.DataType.INT8) {
                            quantized = quantizedOutputBuffer[i][j][k];
                        }
                        outputBuffer[i][j][k] = (quantized - outputZeroPoint) * outputScale;
                    }
                }
            }
        }

        private void parseDetections(int originalWidth, int originalHeight) {
            List<Detection> candidateDetections = new ArrayList<>();
            int[] outputShape = interpreter.getOutputTensor(0).shape();

            boolean isTransposed = (outputShape[1] == numBoxes);

            int numClasses = labels.length;

            for (int i = 0; i < numBoxes; i++) {
                float centerX;
                float centerY;
                float width;
                float height;
                float[] classScores = new float[numClasses];

                if (isTransposed) {
                    centerX = outputBuffer[0][i][0];
                    centerY = outputBuffer[0][i][1];
                    width = outputBuffer[0][i][2];
                    height = outputBuffer[0][i][3];
                    for (int j = 0; j < numClasses; j++) {
                        classScores[j] = outputBuffer[0][i][4 + j];
                    }
                } else {
                    centerX = outputBuffer[0][0][i];
                    centerY = outputBuffer[0][1][i];
                    width = outputBuffer[0][2][i];
                    height = outputBuffer[0][3][i];
                    for (int j = 0; j < numClasses; j++) {
                        classScores[j] = outputBuffer[0][4 + j][i];
                    }
                }

                int maxClassIdx = 0;
                float maxConfidence = classScores[0];
                for (int j = 1; j < numClasses; j++) {
                    if (classScores[j] > maxConfidence) {
                        maxConfidence = classScores[j];
                        maxClassIdx = j;
                    }
                }

                if (maxConfidence < CONFIDENCE_THRESHOLD) {
                    continue;
                }

                float scaleX = (float) originalWidth / inputWidth;
                float scaleY = (float) originalHeight / inputHeight;

                float x = (centerX - width / 2) * scaleX;
                float y = (centerY - height / 2) * scaleY;
                float w = width * scaleX;
                float h = height * scaleY;

                if (x < 0 || y < 0 || w <= 0 || h <= 0) {
                    continue;
                }

                String label = maxClassIdx < labels.length ? labels[maxClassIdx] : "unknown";
                candidateDetections.add(new Detection(label, maxConfidence, x, y, w, h));
            }

            detections.addAll(nms(candidateDetections));
        }

        private List<Detection> nms(List<Detection> boxes) {
            if (boxes.isEmpty()) {
                return new ArrayList<>();
            }

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
                    Scalar color = new Scalar(0, 255, 0);
                    if ("purpleball".equals(detection.label)) {
                        color = new Scalar(128, 0, 128);
                    } else if ("greenball".equals(detection.label)) {
                        color = new Scalar(0, 255, 0);
                    }

                    Point topLeft = new Point(detection.x, detection.y);
                    Point bottomRight = new Point(detection.x + detection.width, detection.y + detection.height);
                    Imgproc.rectangle(frame, topLeft, bottomRight, color, 2);

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
