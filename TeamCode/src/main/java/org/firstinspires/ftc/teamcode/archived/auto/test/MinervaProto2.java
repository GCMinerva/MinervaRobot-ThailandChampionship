package org.firstinspires.ftc.teamcode.archived.auto.test;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

/**
 * Enhanced Autonomous with AprilTag Detection and Color Sorting
 *
 * Features:
 * - AprilTag detection (IDs 21, 22, 23) at the start
 * - Color sensor-based ball sorting (green/purple)
 * - Dynamic speed control (max speed for movement, 0.4 for collection)
 * - Ball memory system (tracks up to 3 balls, resets at "infrontball" paths)
 * - Shooting sequence with 6-second stop and pattern-based ball ejection
 */
@Autonomous(name = "MinervaProto2", group = "Test")
public class MinervaProto2 extends LinearOpMode {

    // Robot hardware
    private Follower follower;
    private RobotConfig robot;

    // Custom hardware not in RobotConfig
    private Servo sorterServo;          // Axon servo "sorter" for ball sorting
    private Servo binitServo;           // Servo "binit" for shooting mechanism
    private ColorSensor colorSensor;    // Color sensor "colorsen"

    // AprilTag detection
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private int detectedTagId = -1;     // Stores detected tag ID (21, 22, or 23)

    // Ball tracking system
    private enum BallColor { GREEN, PURPLE, NONE }
    private List<BallColor> ballMemory = new ArrayList<>();
    private int ballCount = 0;
    private static final int MAX_BALLS = 3;

    // Sorter servo positions
    private static final double SORTER_TRAY_1 = 0.0;   // First tray (initial position)
    private static final double SORTER_TRAY_2 = 0.4;   // Second tray
    private static final double SORTER_TRAY_3 = 0.75;  // Third tray

    // Sorter shooting positions
    private static final double SORTER_SHOOT_1 = 0.55; // Shoot from tray 1
    private static final double SORTER_SHOOT_2 = 0.9;  // Shoot from tray 2
    private static final double SORTER_SHOOT_3 = 0.15; // Shoot from tray 3

    // Path state machine
    private int pathState = 0;
    private PathChain currentPath;
    private String currentPathName = "";

    // Timers
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime shootTimer = new ElapsedTime();
    private Timer pathTimer = new Timer();

    // Shooting state machine
    private int shootingState = 0;

    // Speed control
    private static final double MAX_SPEED = 1.0;
    private static final double COLLECT_SPEED = 0.4;

    // Starting pose
    private final Pose startPose = new Pose(58.706, 0.425, Math.toRadians(90));

    @Override
    public void runOpMode() {
        // Initialize hardware
        initializeHardware();

        // Initialize AprilTag detection
        initAprilTag();

        telemetry.addData("Status", "Initialized - Detecting AprilTag...");
        telemetry.update();

        // Wait for start and detect AprilTag
        while (!isStarted() && !isStopRequested()) {
            detectAprilTag();
            telemetry.addData("AprilTag", detectedTagId == -1 ? "Not detected" : "ID: " + detectedTagId);
            telemetry.addData("Ready", "Press START to begin");
            telemetry.update();
        }

        waitForStart();
        runtime.reset();

        // Set sorter servo to tray 1 (position 0) at start
        sorterServo.setPosition(SORTER_TRAY_1);

        // Main autonomous loop
        while (opModeIsActive()) {
            // Update follower
            follower.update();

            // Update path state machine
            updatePathStateMachine();

            // Check for ball colors during collection
            if (currentPathName.contains("colectball")) {
                checkColorSensor();
            }

            // Telemetry
            updateTelemetry();
        }
    }

    /**
     * Initialize all hardware components
     */
    private void initializeHardware() {
        // Initialize robot config
        robot = new RobotConfig(hardwareMap);

        // Initialize Pedro Pathing follower
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        // Initialize custom hardware
        sorterServo = hardwareMap.get(Servo.class, "sorter");
        binitServo = hardwareMap.get(Servo.class, "binit");
        colorSensor = hardwareMap.get(ColorSensor.class, "colorsen");

        // Set initial positions
        sorterServo.setPosition(SORTER_TRAY_1);  // Start at tray 1 (position 0)
        binitServo.setPosition(0);
    }

    /**
     * Initialize AprilTag processor
     */
    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder().build();
        aprilTag.setDecimation(2);

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();
    }

    /**
     * Detect AprilTag and save ID if it's 21, 22, or 23
     */
    private void detectAprilTag() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                int id = detection.id;
                if (id == 21 || id == 22 || id == 23) {
                    detectedTagId = id;
                    return;
                }
            }
        }
    }

    /**
     * Path state machine - manages path execution
     */
    private void updatePathStateMachine() {
        switch (pathState) {
            case 0: // init-ball-shoot
                currentPathName = "init-ball-shoot";
                currentPath = follower.pathBuilder()
                        .addPath(new BezierLine(new Pose(58.706, 0.425), new Pose(59.131, 14.251)))
                        .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(110))
                        .build();
                setMaxSpeed();
                follower.followPath(currentPath, true);
                pathState++;
                break;

            case 1: // Wait for path completion
                if (!follower.isBusy()) {
                    pathState++;
                }
                break;

            case 2: // infrontball1 - reset ball memory
                currentPathName = "infrontball1";
                resetBallMemory();
                currentPath = follower.pathBuilder()
                        .addPath(new BezierCurve(
                                new Pose(59.131, 14.251),
                                new Pose(57.217, 30.842),
                                new Pose(41.690, 29.140)
                        ))
                        .setLinearHeadingInterpolation(Math.toRadians(110), Math.toRadians(180))
                        .build();
                setMaxSpeed();
                follower.followPath(currentPath, true);
                pathState++;
                break;

            case 3: // Wait
                if (!follower.isBusy()) {
                    pathState++;
                }
                break;

            case 4: // colectball1
                currentPathName = "colectball1";
                currentPath = follower.pathBuilder()
                        .addPath(new BezierLine(new Pose(41.690, 29.140), new Pose(17.867, 28.502)))
                        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                        .build();
                setCollectSpeed();
                follower.followPath(currentPath, true);
                pathState++;
                break;

            case 5: // Wait
                if (!follower.isBusy()) {
                    pathState++;
                }
                break;

            case 6: // backtoshootpoint1
                currentPathName = "backtoshootpoint1";
                currentPath = follower.pathBuilder()
                        .addPath(new BezierLine(new Pose(17.867, 28.502), new Pose(59.557, 14.038)))
                        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(110))
                        .build();
                setMaxSpeed();
                follower.followPath(currentPath, true);
                pathState++;
                break;

            case 7: // Wait
                if (!follower.isBusy()) {
                    pathState++;
                }
                break;

            case 8: // toshooterpoint1 - execute shooting sequence
                currentPathName = "toshooterpoint1";
                executeShootingSequence();
                pathState++;
                break;

            case 9: // infrontball2 - reset ball memory
                currentPathName = "infrontball2";
                resetBallMemory();
                currentPath = follower.pathBuilder()
                        .addPath(new BezierLine(new Pose(59.557, 14.038), new Pose(43.179, 55.090)))
                        .setLinearHeadingInterpolation(Math.toRadians(110), Math.toRadians(180))
                        .build();
                setMaxSpeed();
                follower.followPath(currentPath, true);
                pathState++;
                break;

            case 10: // Wait
                if (!follower.isBusy()) {
                    pathState++;
                }
                break;

            case 11: // colectball2
                currentPathName = "colectball2";
                currentPath = follower.pathBuilder()
                        .addPath(new BezierLine(new Pose(43.179, 55.090), new Pose(23.610, 55.516)))
                        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                        .build();
                setCollectSpeed();
                follower.followPath(currentPath, true);
                pathState++;
                break;

            case 12: // Wait
                if (!follower.isBusy()) {
                    pathState++;
                }
                break;

            case 13: // toshooterpoint2
                currentPathName = "toshooterpoint2";
                currentPath = follower.pathBuilder()
                        .addPath(new BezierLine(new Pose(23.610, 55.516), new Pose(59.557, 75.510)))
                        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                        .build();
                setMaxSpeed();
                follower.followPath(currentPath, true);
                pathState++;
                break;

            case 14: // Wait
                if (!follower.isBusy()) {
                    pathState++;
                }
                break;

            case 15: // Execute shooting sequence 2
                currentPathName = "toshooterpoint2";
                executeShootingSequence();
                pathState++;
                break;

            case 16: // infrontball3 - reset ball memory
                currentPathName = "infrontball3";
                resetBallMemory();
                currentPath = follower.pathBuilder()
                        .addPath(new BezierLine(new Pose(59.557, 75.510), new Pose(42.753, 80.189)))
                        .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                        .build();
                setMaxSpeed();
                follower.followPath(currentPath, true);
                pathState++;
                break;

            case 17: // Wait
                if (!follower.isBusy()) {
                    pathState++;
                }
                break;

            case 18: // colectball3open
                currentPathName = "colectball3open";
                currentPath = follower.pathBuilder()
                        .addPath(new BezierCurve(
                                new Pose(42.753, 80.189),
                                new Pose(28.928, 87.634),
                                new Pose(16.378, 70.405)
                        ))
                        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                        .build();
                setCollectSpeed();
                follower.followPath(currentPath, true);
                pathState++;
                break;

            case 19: // Wait
                if (!follower.isBusy()) {
                    pathState++;
                }
                break;

            case 20: // toshooterpoint3
                currentPathName = "toshooterpoint3";
                currentPath = follower.pathBuilder()
                        .addPath(new BezierLine(new Pose(16.378, 70.405), new Pose(48.000, 94.000)))
                        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                        .build();
                setMaxSpeed();
                follower.followPath(currentPath, true);
                pathState++;
                break;

            case 21: // Wait
                if (!follower.isBusy()) {
                    pathState++;
                }
                break;

            case 22: // Execute final shooting sequence
                currentPathName = "toshooterpoint3";
                executeShootingSequence();
                pathState++;
                break;

            case 23: // Autonomous complete
                currentPathName = "Complete";
                break;
        }
    }

    /**
     * Check color sensor and handle ball detection
     * First ball stays at tray 1 (0.0), second ball moves to tray 2 (0.4), third ball moves to tray 3 (0.75)
     */
    private void checkColorSensor() {
        if (ballCount >= MAX_BALLS) {
            return; // Already have 3 balls
        }

        int red = colorSensor.red();
        int green = colorSensor.green();
        int blue = colorSensor.blue();

        // Determine ball color based on sensor values
        BallColor detectedColor = BallColor.NONE;

        // Green ball detection (more green than red/blue)
        if (green > red && green > blue && green > 100) {
            detectedColor = BallColor.GREEN;
        }
        // Purple ball detection (high red and blue, low green)
        else if (red > 100 && blue > 100 && green < red * 0.5) {
            detectedColor = BallColor.PURPLE;
        }

        // If a ball is detected, add to memory and move sorter to next tray
        if (detectedColor != BallColor.NONE) {
            ballMemory.add(detectedColor);
            ballCount++;

            // Move sorter to appropriate tray based on ball count
            switch (ballCount) {
                case 1:
                    // First ball stays at tray 1, no movement needed
                    sorterServo.setPosition(SORTER_TRAY_1);
                    break;
                case 2:
                    // Second ball detected, move to tray 2
                    sorterServo.setPosition(SORTER_TRAY_2);
                    break;
                case 3:
                    // Third ball detected, move to tray 3
                    sorterServo.setPosition(SORTER_TRAY_3);
                    break;
            }

            // Brief pause to allow servo to move
            sleep(300);
        }
    }

    /**
     * Reset ball memory at "infrontball" paths
     */
    private void resetBallMemory() {
        ballMemory.clear();
        ballCount = 0;
        sorterServo.setPosition(SORTER_TRAY_1);  // Reset to tray 1 (position 0)
    }

    /**
     * Execute shooting sequence with 6-second stop
     */
    private void executeShootingSequence() {
        shootTimer.reset();
        shootingState = 0;

        // Check if we have valid balls to shoot
        boolean hasValidPattern = validateBallPattern();

        // 6-second shooting routine
        while (shootTimer.seconds() < 6.0 && opModeIsActive()) {
            follower.update(); // Keep follower updated

            // Spin outtake motor at 0.7 speed
            robot.shooter.outtakeMotor.setPower(0.7);

            // Shoot balls according to pattern
            if (hasValidPattern) {
                shootBallsInPattern();
            } else {
                // If invalid pattern, just shoot all balls
                shootAllBalls();
            }

            updateTelemetry();
        }

        // Stop outtake motor after 6 seconds
        robot.shooter.outtakeMotor.setPower(0);
    }

    /**
     * Validate ball pattern based on AprilTag ID
     * Returns false if pattern doesn't match expected format
     */
    private boolean validateBallPattern() {
        if (ballMemory.size() != 3) {
            return false;
        }

        // Check if all balls are purple (invalid pattern)
        boolean allPurple = true;
        for (BallColor color : ballMemory) {
            if (color != BallColor.PURPLE) {
                allPurple = false;
                break;
            }
        }

        if (allPurple) {
            return false; // All purple is invalid
        }

        // Check expected pattern based on AprilTag ID
        switch (detectedTagId) {
            case 21: // ID:21 - Green first, then 2 purple
                return ballMemory.get(0) == BallColor.GREEN;

            case 22: // ID:22 - Purple second (green, purple, green)
                return ballMemory.get(1) == BallColor.PURPLE
                    && ballMemory.get(0) == BallColor.GREEN
                    && ballMemory.get(2) == BallColor.GREEN;

            case 23: // ID:23 - Purple first 2, green last
                return ballMemory.get(0) == BallColor.PURPLE
                    && ballMemory.get(1) == BallColor.PURPLE;

            default:
                return false;
        }
    }

    /**
     * Shoot balls in pattern based on AprilTag ID
     * Uses sorter positions: 0.55 for tray 1, 0.9 for tray 2, 0.15 for tray 3
     */
    private void shootBallsInPattern() {
        List<Integer> shootOrder = new ArrayList<>();

        // Determine shoot order based on AprilTag ID
        switch (detectedTagId) {
            case 21: // Green first, then purple balls
                for (int i = 0; i < ballMemory.size(); i++) {
                    if (ballMemory.get(i) == BallColor.GREEN) {
                        shootOrder.add(0, i); // Green balls first
                    } else {
                        shootOrder.add(i); // Purple balls after
                    }
                }
                break;

            case 22: // Purple second (shoot in order: 0, 2, 1)
                shootOrder.add(0); // First green
                shootOrder.add(2); // Third green
                shootOrder.add(1); // Second purple
                break;

            case 23: // Purple first 2, green last (shoot in order: 2, 0, 1)
                shootOrder.add(2); // Third (green)
                shootOrder.add(0); // First (purple)
                shootOrder.add(1); // Second (purple)
                break;
        }

        // Execute shooting sequence
        for (int ballIndex : shootOrder) {
            shootSingleBall(ballIndex);
        }
    }

    /**
     * Shoot all balls without pattern checking
     */
    private void shootAllBalls() {
        for (int i = 0; i < ballMemory.size(); i++) {
            shootSingleBall(i);
        }
    }

    /**
     * Shoot a single ball from specified tray
     * @param trayIndex 0 = tray 1, 1 = tray 2, 2 = tray 3
     * Moves sorter to shooting position for that tray:
     * - Tray 1: 0.55
     * - Tray 2: 0.9
     * - Tray 3: 0.15
     */
    private void shootSingleBall(int trayIndex) {
        // Move sorter to shooting position for the specified tray
        double shootPosition;
        switch (trayIndex) {
            case 0:
                shootPosition = SORTER_SHOOT_1;  // 0.55 for tray 1
                break;
            case 1:
                shootPosition = SORTER_SHOOT_2;  // 0.9 for tray 2
                break;
            case 2:
                shootPosition = SORTER_SHOOT_3;  // 0.15 for tray 3
                break;
            default:
                shootPosition = SORTER_SHOOT_1;
                break;
        }

        // Set sorter to shooting position
        sorterServo.setPosition(shootPosition);

        // Wait for sorter to move into position
        sleep(400);

        // Set binit servo to 60 degrees (0.33 position for 180-degree servo) to eject ball
        binitServo.setPosition(0.33);

        // Wait for ball to be ejected
        sleep(500);

        // Reset binit servo
        binitServo.setPosition(0);

        // Wait before next ball
        sleep(300);
    }

    /**
     * Set maximum speed for follower
     */
    private void setMaxSpeed() {
        // Pedro Pathing uses path constraints for speed control
        // This is set globally in Constants, but we can override with custom constraints
        follower.setMaxPower(MAX_SPEED);
    }

    /**
     * Set collection speed (0.4) for follower
     */
    private void setCollectSpeed() {
        follower.setMaxPower(COLLECT_SPEED);
    }

    /**
     * Update telemetry display
     */
    private void updateTelemetry() {
        Pose currentPose = follower.getPose();

        telemetry.addData("=== STATUS ===", "");
        telemetry.addData("Runtime", "%.1f s", runtime.seconds());
        telemetry.addData("Path State", pathState);
        telemetry.addData("Current Path", currentPathName);

        telemetry.addData("=== APRILTAG ===", "");
        telemetry.addData("Detected ID", detectedTagId == -1 ? "None" : detectedTagId);

        telemetry.addData("=== BALLS ===", "");
        telemetry.addData("Ball Count", "%d/%d", ballCount, MAX_BALLS);
        for (int i = 0; i < ballMemory.size(); i++) {
            telemetry.addData("Ball " + (i + 1), ballMemory.get(i));
        }
        telemetry.addData("Sorter Position", "%.2f", sorterServo.getPosition());

        telemetry.addData("=== POSITION ===", "");
        telemetry.addData("X", "%.1f", currentPose.getX());
        telemetry.addData("Y", "%.1f", currentPose.getY());
        telemetry.addData("Heading", "%.1f°", Math.toDegrees(currentPose.getHeading()));

        telemetry.update();
    }
}
