package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

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

        @Override
        public Mat processFrame(Mat input) {
            // TODO: Implement YOLO model inference using TensorFlow Lite
            // This is a placeholder implementation
            // You'll need to:
            // 1. Load the yolo.tflite model
            // 2. Preprocess the input Mat
            // 3. Run inference
            // 4. Parse outputs into Detection objects
            // 5. Draw bounding boxes on input

            synchronized (detections) {
                detections.clear();
                // Add detected objects here after implementing inference
            }

            return input;
        }

        public List<Detection> getDetections() {
            synchronized (detections) {
                return new ArrayList<>(detections);
            }
        }
    }
}