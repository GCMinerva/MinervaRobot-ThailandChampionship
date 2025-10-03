package org.firstinspires.ftc.teamcode.tuning;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.ArrayList;
import java.util.Locale;

/**
 * Shooter Tuning OpMode
 * Calibrate your shooter with distance-based power tuning
 *
 * @author MinervaRobot Team
 * @version 1.0
 */
@TeleOp(name = "Shooter Tuning", group = "Tuning")
public class ShooterTuning extends OpMode {

    private RobotConfig robot;
    private Follower follower;

    // Calibration data storage
    private ArrayList<CalibrationPoint> calibrationPoints = new ArrayList<>();

    // Target position
    private double targetX = 11.0;
    private double targetY = 135.0;

    // Current calibration mode
    private CalibrationMode mode = CalibrationMode.MANUAL_SHOOT;

    // Manual power control
    private double manualPower = 0.5;
    private static final double POWER_INCREMENT = 0.05;

    // Button states
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastA = false;
    private boolean lastB = false;
    private boolean lastX = false;
    private boolean lastY = false;
    private boolean lastLeftBumper = false;
    private boolean lastRightBumper = false;

    // Servo state
    private boolean servoExtended = false;

    enum CalibrationMode {
        MANUAL_SHOOT,
        RECORD_POINT,
        VIEW_DATA,
        CALCULATE_CALIBRATION
    }

    static class CalibrationPoint {
        double distance;
        double power;
        Pose position;

        CalibrationPoint(double distance, double power, Pose position) {
            this.distance = distance;
            this.power = power;
            this.position = position;
        }
    }

    @Override
    public void init() {
        // Initialize robot and follower
        robot = new RobotConfig(hardwareMap);
        follower = Constants.createFollower(hardwareMap);

        // Set target position
        robot.shooter.setTargetPosition(targetX, targetY);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Target", "X: %.1f, Y: %.1f", targetX, targetY);
        telemetry.update();
    }

    @Override
    public void loop() {
        // Update follower for position tracking
        follower.update();
        Pose currentPose = follower.getPose();
        double currentDistance = robot.shooter.getDistanceToTarget(currentPose);

        // Mode switching with bumpers
        boolean leftBumper = gamepad1.left_bumper;
        if (leftBumper && !lastLeftBumper) {
            cycleMode(-1);
        }
        lastLeftBumper = leftBumper;

        boolean rightBumper = gamepad1.right_bumper;
        if (rightBumper && !lastRightBumper) {
            cycleMode(1);
        }
        lastRightBumper = rightBumper;

        // Handle different modes
        switch (mode) {
            case MANUAL_SHOOT:
                handleManualShoot(currentPose, currentDistance);
                break;
            case RECORD_POINT:
                handleRecordPoint(currentPose, currentDistance);
                break;
            case VIEW_DATA:
                handleViewData(currentDistance);
                break;
            case CALCULATE_CALIBRATION:
                handleCalculateCalibration(currentDistance);
                break;
        }

        // Display common telemetry
        telemetry.addData("Mode", mode.toString());
        telemetry.addData("", "");
        telemetry.addData("Position", "X: %.1f, Y: %.1f", currentPose.getX(), currentPose.getY());
        telemetry.addData("Distance to Target", "%.1f inches", currentDistance);
        telemetry.addData("Recorded Points", calibrationPoints.size());
        telemetry.addData("", "");
        telemetry.addData("Controls", "LB/RB: Change Mode");
        telemetry.update();
    }

    private void handleManualShoot(Pose currentPose, double currentDistance) {
        // Adjust power with D-pad
        if (gamepad1.dpad_up && !lastDpadUp) {
            manualPower = Math.min(1.0, manualPower + POWER_INCREMENT);
        }
        lastDpadUp = gamepad1.dpad_up;

        if (gamepad1.dpad_down && !lastDpadDown) {
            manualPower = Math.max(0.0, manualPower - POWER_INCREMENT);
        }
        lastDpadDown = gamepad1.dpad_down;

        // A: Set motor power
        boolean aButton = gamepad1.a;
        if (aButton && !lastA) {
            robot.shooter.setShooterPower(manualPower);
            telemetry.addData("Action", "Motors set to %.2f", manualPower);
        }
        lastA = aButton;

        // B: Toggle servo and outtake motor
        boolean bButton = gamepad1.b;
        if (bButton && !lastB) {
            if (!servoExtended) {
                // Start shooting - set servo and start outtake
                robot.shooter.setServoPosition(0.9);
                robot.shooter.feedBall();  // Sets outtake to 1.0
                servoExtended = true;
            } else {
                // Stop shooting - reset servo and stop outtake
                robot.shooter.setServoPosition(0.0);
                robot.shooter.stopFeed();
                servoExtended = false;
            }
        }
        lastB = bButton;

        // X: Stop motors
        boolean xButton = gamepad1.x;
        if (xButton && !lastX) {
            robot.shooter.stopAll();
            telemetry.addData("Action", "All motors stopped");
        }
        lastX = xButton;

        // Y: Record this power/distance
        boolean yButton = gamepad1.y;
        if (yButton && !lastY) {
            calibrationPoints.add(new CalibrationPoint(currentDistance, manualPower,
                new Pose(currentPose.getX(), currentPose.getY(), currentPose.getHeading())));
            telemetry.addData("Action", "Point recorded!");
        }
        lastY = yButton;

        telemetry.addData("Manual Power", "%.2f", manualPower);
        telemetry.addData("Servo", servoExtended ? "SHOOT (0.9)" : "RESET (0.0)");
        telemetry.addData("Outtake", servoExtended ? "ON (1.0)" : "OFF");
        telemetry.addData("", "");
        telemetry.addData("D-Pad Up/Down", "Adjust Power");
        telemetry.addData("A", "Set Shooter Power");
        telemetry.addData("B", "Toggle Servo & Outtake");
        telemetry.addData("X", "Stop All Motors");
        telemetry.addData("Y", "Record Point");
    }

    private void handleRecordPoint(Pose currentPose, double currentDistance) {
        // A: Auto-test current position
        boolean aButton = gamepad1.a;
        if (aButton && !lastA) {
            // Use current calculated power from shooter
            double autoPower = robot.shooter.getCalculatedPower(currentPose);
            robot.shooter.setShooterPower(autoPower);
            telemetry.addData("Action", "Testing with power %.2f", autoPower);
        }
        lastA = aButton;

        // B: Record with manual power
        boolean bButton = gamepad1.b;
        if (bButton && !lastB) {
            calibrationPoints.add(new CalibrationPoint(currentDistance, manualPower,
                new Pose(currentPose.getX(), currentPose.getY(), currentPose.getHeading())));
            telemetry.addData("Action", "Recorded: %.1f in @ %.2f pwr", currentDistance, manualPower);
        }
        lastB = bButton;

        // X: Clear last point
        boolean xButton = gamepad1.x;
        if (xButton && !lastX && !calibrationPoints.isEmpty()) {
            CalibrationPoint removed = calibrationPoints.remove(calibrationPoints.size() - 1);
            telemetry.addData("Action", "Removed: %.1f in", removed.distance);
        }
        lastX = xButton;

        // Y: Clear all points
        boolean yButton = gamepad1.y;
        if (yButton && !lastY) {
            calibrationPoints.clear();
            telemetry.addData("Action", "All points cleared!");
        }
        lastY = yButton;

        telemetry.addData("Manual Power", "%.2f", manualPower);
        telemetry.addData("", "");
        telemetry.addData("A", "Test Auto Power");
        telemetry.addData("B", "Record Manual Point");
        telemetry.addData("X", "Remove Last Point");
        telemetry.addData("Y", "Clear All Points");
    }

    private void handleViewData(double currentDistance) {
        telemetry.addData("Calibration Points", "");

        if (calibrationPoints.isEmpty()) {
            telemetry.addData("", "No points recorded yet!");
        } else {
            for (int i = 0; i < calibrationPoints.size(); i++) {
                CalibrationPoint p = calibrationPoints.get(i);
                telemetry.addData(String.format(Locale.US, "[%d]", i),
                    "Dist: %.1f, Pwr: %.2f", p.distance, p.power);
            }
        }

        telemetry.addData("", "");
        telemetry.addData("X", "Clear All Data");

        boolean xButton = gamepad1.x;
        if (xButton && !lastX) {
            calibrationPoints.clear();
        }
        lastX = xButton;
    }

    private void handleCalculateCalibration(double currentDistance) {
        if (calibrationPoints.size() < 2) {
            telemetry.addData("Error", "Need at least 2 points!");
            telemetry.addData("", "Record more points first");
            return;
        }

        // Find min and max distance points
        CalibrationPoint minPoint = calibrationPoints.get(0);
        CalibrationPoint maxPoint = calibrationPoints.get(0);

        for (CalibrationPoint p : calibrationPoints) {
            if (p.distance < minPoint.distance) minPoint = p;
            if (p.distance > maxPoint.distance) maxPoint = p;
        }

        // Calculate average powers at min and max
        double minPowerSum = 0;
        int minCount = 0;
        double maxPowerSum = 0;
        int maxCount = 0;

        double tolerance = 3.0; // 3 inch tolerance

        for (CalibrationPoint p : calibrationPoints) {
            if (Math.abs(p.distance - minPoint.distance) < tolerance) {
                minPowerSum += p.power;
                minCount++;
            }
            if (Math.abs(p.distance - maxPoint.distance) < tolerance) {
                maxPowerSum += p.power;
                maxCount++;
            }
        }

        double avgMinPower = minPowerSum / minCount;
        double avgMaxPower = maxPowerSum / maxCount;

        telemetry.addData("=== CALIBRATION RESULTS ===", "");
        telemetry.addData("", "");
        telemetry.addData("Min Distance", "%.1f inches", minPoint.distance);
        telemetry.addData("Min Power", "%.3f", avgMinPower);
        telemetry.addData("", "");
        telemetry.addData("Max Distance", "%.1f inches", maxPoint.distance);
        telemetry.addData("Max Power", "%.3f", avgMaxPower);
        telemetry.addData("", "");
        telemetry.addData("Code to add:", "");
        telemetry.addData("", "shooter.setCalibration(");
        telemetry.addData("", "    %.1f, %.3f,", minPoint.distance, avgMinPower);
        telemetry.addData("", "    %.1f, %.3f", maxPoint.distance, avgMaxPower);
        telemetry.addData("", ");");

        // A: Apply calibration temporarily
        boolean aButton = gamepad1.a;
        if (aButton && !lastA) {
            robot.shooter.setCalibration(minPoint.distance, avgMinPower,
                                        maxPoint.distance, avgMaxPower);
            telemetry.addData("", "");
            telemetry.addData("Status", "APPLIED!");
        }
        lastA = aButton;

        telemetry.addData("", "");
        telemetry.addData("A", "Apply Calibration");
    }

    private void cycleMode(int direction) {
        int ordinal = mode.ordinal() + direction;
        if (ordinal < 0) ordinal = CalibrationMode.values().length - 1;
        if (ordinal >= CalibrationMode.values().length) ordinal = 0;
        mode = CalibrationMode.values()[ordinal];
    }
}
