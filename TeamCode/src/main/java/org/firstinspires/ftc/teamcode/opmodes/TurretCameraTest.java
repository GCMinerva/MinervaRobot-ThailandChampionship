package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.archived.subsystems.TurretAprilTagSubsystem;
import org.firstinspires.ftc.teamcode.RobotHardwareConfig;

/**
 * TurretCameraTest - Test OpMode for AprilTag-based turret tracking
 * 
 * This OpMode demonstrates how to use the TurretAprilTagSubsystem
 * to track AprilTags on goals and control the turret.
 * 
 * Configuration:
 * - Set targetAprilTagId to the tag you want to track
 * - Camera must be mounted (upside-down is supported)
 * - Turret motor must be configured as "turret"
 * 
 * Controls:
 * - Gamepad2 dpad_left: Toggle tracking ON/OFF
 * - Gamepad2 dpad_up: Set target tag to 11 (Red goal example)
 * - Gamepad2 dpad_down: Set target tag to 12 (Blue goal example)
 * - Gamepad2 right_stick_y: Manual turret control (when tracking OFF)
 * 
 * Telemetry shows:
 * - Tag detection status
 * - Bearing angle to tag
 * - Distance to tag
 * - Turret power
 */
@TeleOp(name = "Turret Camera Test", group = "Test")
public class TurretCameraTest extends OpMode {

    // Subsystems
    private TurretAprilTagSubsystem turretAprilTag;
    private DcMotor turret;

    // Drive motors
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    // Configuration - CHANGE THESE VALUES
    private int targetAprilTagId = 24; // Default: Tag 24 (Red goal)
    // Tag 20 = Blue goal
    // Tag 24 = Red goal

    // Turret starting angle (set this to match physical position at init)
    private double turretStartingAngle = 0.0; // 0° = straight ahead, +90° = right, -90° = left

    // State
    private boolean trackingEnabled = true;
    private boolean dpadLeftPressed = false;
    private boolean dpadUpPressed = false;
    private boolean dpadDownPressed = false;

    // Control parameters
    private double turretPower = 0.0;
    private static final double TICKS_PER_DEGREE = 6.208; // From TurretSubsystem calibration
    private static final double MIN_ANGLE = -90.0; // Left limit
    private static final double MAX_ANGLE = 90.0; // Right limit
    private static final double TRACKING_POWER = 0.5; // Power when tracking

    @Override
    public void init() {
        // Initialize AprilTag subsystem
        turretAprilTag = new TurretAprilTagSubsystem(hardwareMap, telemetry);

        // Set target AprilTag ID
        setTargetAprilTag(targetAprilTagId);

        // Initialize turret motor with encoder
        turret = hardwareMap.get(DcMotor.class, "turret");
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reset encoder and set to RUN_TO_POSITION mode
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setTargetPosition(0);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setPower(0.0);

        // Initialize drive motors
        frontLeft = hardwareMap.get(DcMotor.class, RobotHardwareConfig.DriveMotors.FRONT_LEFT);
        frontRight = hardwareMap.get(DcMotor.class, RobotHardwareConfig.DriveMotors.FRONT_RIGHT);
        backLeft = hardwareMap.get(DcMotor.class, RobotHardwareConfig.DriveMotors.BACK_LEFT);
        backRight = hardwareMap.get(DcMotor.class, RobotHardwareConfig.DriveMotors.BACK_RIGHT);

        frontLeft.setDirection(RobotHardwareConfig.DriveDirections.FRONT_LEFT);
        frontRight.setDirection(RobotHardwareConfig.DriveDirections.FRONT_RIGHT);
        backLeft.setDirection(RobotHardwareConfig.DriveDirections.BACK_LEFT);
        backRight.setDirection(RobotHardwareConfig.DriveDirections.BACK_RIGHT);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set turret to starting angle
        setTurretAngle(turretStartingAngle);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Target Tag", targetAprilTagId);
        telemetry.addData("Tracking", trackingEnabled ? "ENABLED" : "DISABLED");
        telemetry.addData("Turret Mode", "Encoder (RUN_TO_POSITION)");
        telemetry.addData("Starting Angle", "%.1f°", turretStartingAngle);
        telemetry.update();
    }

    @Override
    public void loop() {
        // === MECANUM DRIVE CONTROL (Gamepad1) ===
        double forwardSpeed = -gamepad1.left_stick_y; // Forward/backward
        double strafeSpeed = gamepad1.left_stick_x; // Left/right strafe
        double turnSpeed = gamepad1.right_stick_x; // Rotation

        // Calculate mecanum drive motor powers
        double frontLeftPower = forwardSpeed + strafeSpeed + turnSpeed;
        double frontRightPower = forwardSpeed - strafeSpeed - turnSpeed;
        double backLeftPower = forwardSpeed - strafeSpeed + turnSpeed;
        double backRightPower = forwardSpeed + strafeSpeed - turnSpeed;

        // Normalize powers to ensure they don't exceed 1.0
        double maxPower = Math.max(Math.abs(frontLeftPower),
                Math.max(Math.abs(frontRightPower),
                        Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))));

        if (maxPower > 1.0) {
            frontLeftPower /= maxPower;
            frontRightPower /= maxPower;
            backLeftPower /= maxPower;
            backRightPower /= maxPower;
        }

        // Set motor powers
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);

        // === GAMEPAD CONTROLS (Gamepad2) ===

        // Toggle tracking with dpad_left
        if (gamepad2.dpad_left && !dpadLeftPressed) {
            dpadLeftPressed = true;
            trackingEnabled = !trackingEnabled;

            if (!trackingEnabled) {
                // Return to center when tracking disabled
                setTurretAngle(0);
            }
        }
        if (!gamepad2.dpad_left)
            dpadLeftPressed = false;

        // Set target tag to 24 (Red goal) with dpad_up
        if (gamepad2.dpad_up && !dpadUpPressed) {
            dpadUpPressed = true;
            setTargetAprilTag(24); // Red goal
        }
        if (!gamepad2.dpad_up)
            dpadUpPressed = false;

        // Set target tag to 20 (Blue goal) with dpad_down
        if (gamepad2.dpad_down && !dpadDownPressed) {
            dpadDownPressed = true;
            setTargetAprilTag(20); // Blue goal
        }
        if (!gamepad2.dpad_down)
            dpadDownPressed = false;

        // === TURRET CONTROL ===

        if (trackingEnabled) {
            // Update AprilTag detection
            turretAprilTag.update();

            if (turretAprilTag.isTagDetected()) {
                // Get bearing angle from AprilTag
                double bearing = turretAprilTag.getBearingDegrees();

                // Get current turret angle from encoder
                int currentTicks = turret.getCurrentPosition();
                double currentAngle = currentTicks / TICKS_PER_DEGREE;

                // Calculate target angle (invert bearing to fix direction)
                // Negative bearing because: tag right = turn right (positive angle)
                double targetAngle = currentAngle - bearing;

                // Set turret to target angle (with limits)
                setTurretAngle(targetAngle);

                turretPower = TRACKING_POWER;
            } else {
                // No tag detected, hold current position
                turret.setPower(0.1); // Low holding power
                turretPower = 0.1;
            }
        } else {
            // Manual control with right stick
            double manualInput = -gamepad2.right_stick_y;

            if (Math.abs(manualInput) > 0.1) {
                // Switch to RUN_USING_ENCODER for manual control
                if (turret.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
                    turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
                turret.setPower(manualInput * 0.5);
                turretPower = manualInput * 0.5;
            } else {
                // Hold position when stick is centered
                if (turret.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
                    int currentPos = turret.getCurrentPosition();
                    turret.setTargetPosition(currentPos);
                    turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
                turret.setPower(0.1);
                turretPower = 0.1;
            }
        }

        // === TELEMETRY ===

        telemetry.addData("=== TURRET TRACKING ===", "");
        telemetry.addData("Tracking Mode", trackingEnabled ? "AUTO (AprilTag)" : "MANUAL");
        telemetry.addData("Target Tag ID", targetAprilTagId);
        telemetry.addData("", "");

        if (trackingEnabled) {
            turretAprilTag.addTelemetry();
        }

        telemetry.addData("=== TURRET ===", "");
        telemetry.addData("Power", "%.2f", turretPower);
        telemetry.addData("", "");

        telemetry.addData("=== CONTROLS ===", "");
        telemetry.addData("GP1 Left Stick", "Drive (Forward/Strafe)");
        telemetry.addData("GP1 Right Stick", "Rotate");
        telemetry.addData("GP2 Dpad Left", "Toggle Tracking");
        telemetry.addData("GP2 Dpad Up", "Target Tag 24 (Red)");
        telemetry.addData("GP2 Dpad Down", "Target Tag 20 (Blue)");
        telemetry.addData("GP2 Right Stick", "Manual Turret");

        telemetry.update();
    }

    @Override
    public void stop() {
        turret.setPower(0);
        turretAprilTag.stop();
    }

    /**
     * Configure target AprilTag ID
     * 
     * @param tagId AprilTag ID to track (e.g., 24 for Red, 20 for Blue)
     */
    private void setTargetAprilTag(int tagId) {
        targetAprilTagId = tagId;
        turretAprilTag.setTargetTagId(tagId);
    }

    /**
     * Set turret to specific angle using encoder ticks
     * 
     * @param angleDegrees Target angle in degrees (-90 to +90)
     */
    private void setTurretAngle(double angleDegrees) {
        // Clamp to physical limits
        double clampedAngle = Math.max(MIN_ANGLE, Math.min(MAX_ANGLE, angleDegrees));

        // Convert to encoder ticks
        int targetTicks = (int) (clampedAngle * TICKS_PER_DEGREE);

        // Ensure we're in RUN_TO_POSITION mode
        if (turret.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
            turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        // Set target position
        turret.setTargetPosition(targetTicks);
        turret.setPower(TRACKING_POWER);
    }
}
