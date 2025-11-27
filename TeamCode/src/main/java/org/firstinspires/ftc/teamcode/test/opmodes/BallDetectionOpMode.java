//package org.firstinspires.ftc.teamcode.opmodes;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.teamcode.vision.BallDetectionProcessor;
//import org.firstinspires.ftc.vision.VisionPortal;
//
//import java.util.List;
//
//@TeleOp(name = "Ball Detection OpMode", group = "Vision")
//public class BallDetectionOpMode extends LinearOpMode {
//
//    private VisionPortal visionPortal;
//    private BallDetectionProcessor ballDetector;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        // Initialize ball detector
//        ballDetector = new BallDetectionProcessor();
//
//        // Initialize vision portal
//        visionPortal = new VisionPortal.Builder()
//            .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
//            .addProcessor(ballDetector)
//            .build();
//
//        telemetry.addLine("Ball Detection Initialized");
//        telemetry.addLine("Press Play to start");
//        telemetry.update();
//
//        waitForStart();
//
//        while (opModeIsActive()) {
//            // Check for errors
//            String error = ballDetector.getLastError();
//            if (!error.isEmpty()) {
//                telemetry.addLine("ERROR: " + error);
//                telemetry.addLine();
//            }
//
//            // Debug info
//            telemetry.addLine(ballDetector.getDebugInfo());
//            telemetry.addLine();
//
//            // Get all detections
//            List<BallDetectionProcessor.Detection> detections = ballDetector.getDetections();
//
//            // Display telemetry
//            telemetry.addData("Total Detections", detections.size());
//            telemetry.addLine();
//
//            // Count by type
//            List<BallDetectionProcessor.Detection> purpleBalls =
//                ballDetector.getDetectionsByLabel("purpleball");
//            List<BallDetectionProcessor.Detection> greenBalls =
//                ballDetector.getDetectionsByLabel("greenball");
//
//            telemetry.addData("Purple Balls", purpleBalls.size());
//            telemetry.addData("Green Balls", greenBalls.size());
//            telemetry.addLine();
//
//            // Display individual detections
//            for (int i = 0; i < detections.size() && i < 5; i++) {
//                BallDetectionProcessor.Detection det = detections.get(i);
//                telemetry.addLine(String.format("%d. %s (%.2f%%)",
//                    i + 1,
//                    det.label,
//                    det.confidence * 100));
//                telemetry.addLine(String.format("   Position: (%.0f, %.0f) to (%.0f, %.0f)",
//                    det.boundingBox.left,
//                    det.boundingBox.top,
//                    det.boundingBox.right,
//                    det.boundingBox.bottom));
//            }
//
//            // Get closest ball to center
//            BallDetectionProcessor.Detection closest =
//                ballDetector.getClosestDetectionToCenter(640, 480);
//
//            if (closest != null) {
//                telemetry.addLine();
//                telemetry.addData("Closest Ball", closest.label);
//                telemetry.addData("Confidence", String.format("%.2f%%", closest.confidence * 100));
//            }
//
//            telemetry.update();
//            sleep(50);
//        }
//
//        // Cleanup
//        visionPortal.close();
//    }
//}
