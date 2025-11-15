package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.firstinspires.ftc.teamcode.RobotHardwareConfig;
import org.firstinspires.ftc.teamcode.vision.BallDetectionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.List;

@Autonomous(name = "Ball Detection Auto", group = "Auto")
public class BallDetectionAuto extends LinearOpMode {

    private VisionPortal visionPortal;
    private BallDetectionProcessor ballDetector;
    private RobotConfig robot;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize robot hardware
        robot = new RobotConfig(hardwareMap);

        // Initialize ball detector
        ballDetector = new BallDetectionProcessor();

        // Initialize vision portal using webcam name from config
        visionPortal = new VisionPortal.Builder()
            .setCamera(hardwareMap.get(WebcamName.class, RobotHardwareConfig.Camera.WEBCAM_NAME))
            .addProcessor(ballDetector)
            .build();

        telemetry.addLine("Ball Detection Auto Initialized");
        telemetry.addLine("Waiting for start...");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            // Example: Look for purple ball
            telemetry.addLine("Scanning for purple ball...");
            telemetry.update();

            boolean purpleBallFound = false;
            long startTime = System.currentTimeMillis();

            // Scan for 3 seconds
            while (opModeIsActive() && !purpleBallFound &&
                   System.currentTimeMillis() - startTime < 3000) {

                if (ballDetector.isBallDetected("purpleball")) {
                    purpleBallFound = true;
                    telemetry.addLine("Purple ball detected!");
                    telemetry.update();
                }
                sleep(50);
            }

            if (purpleBallFound) {
                // Get purple ball detections
                List<BallDetectionProcessor.Detection> purpleBalls =
                    ballDetector.getDetectionsByLabel("purpleball");

                if (!purpleBalls.isEmpty()) {
                    BallDetectionProcessor.Detection targetBall = purpleBalls.get(0);

                    telemetry.addLine("Approaching purple ball...");
                    telemetry.addData("Confidence", String.format("%.2f%%",
                        targetBall.confidence * 100));
                    telemetry.update();

                    // Example: Activate intake when ball is centered
                    // Using camera resolution from config
                    BallDetectionProcessor.Detection closestBall =
                        ballDetector.getClosestDetectionToCenter(
                            RobotHardwareConfig.Camera.RESOLUTION_WIDTH,
                            RobotHardwareConfig.Camera.RESOLUTION_HEIGHT);

                    if (closestBall != null && closestBall.label.equals("purpleball")) {
                        // Calculate center of bounding box
                        float ballCenterX = (closestBall.boundingBox.left +
                                           closestBall.boundingBox.right) / 2.0f;

                        // Simple alignment logic (you would integrate with your drivetrain)
                        if (ballCenterX < 280) {
                            telemetry.addLine("Ball is left - adjust");
                        } else if (ballCenterX > 360) {
                            telemetry.addLine("Ball is right - adjust");
                        } else {
                            telemetry.addLine("Ball is centered!");

                            // TODO: Activate intake when intake subsystem is added
                            // robot.runIntake(RobotConfig.INTAKE_POWER);
                            // robot.activateIntakeServo();

                            sleep(2000);

                            // robot.runIntake(0);
                            // robot.restIntakeServo();
                        }
                        telemetry.update();
                    }
                }
            } else {
                telemetry.addLine("No purple ball found");
                telemetry.update();
            }

            // Alternative: Look for green ball
            sleep(1000);
            telemetry.addLine("Scanning for green ball...");
            telemetry.update();

            if (ballDetector.isBallDetected("greenball")) {
                telemetry.addLine("Green ball detected!");

                List<BallDetectionProcessor.Detection> greenBalls =
                    ballDetector.getDetectionsByLabel("greenball");

                telemetry.addData("Green ball count", greenBalls.size());
                telemetry.update();

                sleep(2000);
            }

            // Stop all motors
            robot.stopAllMotors();
        }

        // Cleanup
        visionPortal.close();

        telemetry.addLine("Auto complete");
        telemetry.update();
    }
}
