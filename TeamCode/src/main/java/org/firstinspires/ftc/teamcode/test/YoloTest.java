package org.firstinspires.ftc.teamcode.test;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@TeleOp(name = "YOLO Test", group = "Test")
public class YoloTest extends LinearOpMode {

    private TfodProcessor tfod;
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {
        // Setup telemetry for both driver station and FTC dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        initTfod();

        telemetry.addData("Status", "Initialized");
        telemetry.addData(">", "Press START to begin");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                telemetryTfod();
                telemetry.update();

                // Control streaming with gamepad
                if (gamepad1.dpad_down) {
                    visionPortal.stopStreaming();
                    telemetry.addData("Camera", "Stopped");
                } else if (gamepad1.dpad_up) {
                    visionPortal.resumeStreaming();
                    telemetry.addData("Camera", "Running");
                }

                sleep(20);
            }
        }

        visionPortal.close();
    }

    private void initTfod() {
        // Create the TensorFlow processor using the YOLO model
        tfod = new TfodProcessor.Builder()
                .setModelAssetName("yolo.tflite")
                .setModelAspectRatio(16.0 / 9.0)
                .setMaxNumRecognitions(10)
                .setIsModelTensorFlow2(true)
                .setIsModelQuantized(true)
                .build();

        // Create the vision portal
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.setCameraResolution(new Size(640, 480));
        builder.addProcessor(tfod);
        builder.enableLiveView(true);

        visionPortal = builder.build();

        // Stream to FTC Dashboard
        FtcDashboard.getInstance().startCameraStream(tfod, 30);
    }

    private void telemetryTfod() {
        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Display info for each recognition
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2;
            double y = (recognition.getTop() + recognition.getBottom()) / 2;

            telemetry.addLine()
                    .addData("Label", recognition.getLabel())
                    .addData("Confidence", String.format("%.2f", recognition.getConfidence()));
            telemetry.addLine()
                    .addData("Position", String.format("%.0f / %.0f", x, y))
                    .addData("Size", String.format("%.0f x %.0f",
                            recognition.getWidth(), recognition.getHeight()));
        }
    }
}