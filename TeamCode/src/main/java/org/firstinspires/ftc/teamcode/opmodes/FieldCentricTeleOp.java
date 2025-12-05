package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.archived.util.GlobalData;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.RobotHardwareConfig;

/**
 * Field-Centric TeleOp with Compass Control
 *
 * This OpMode provides field-centric mecanum drive control using the robot's
 * final pose from autonomous as a reference heading. The driver can toggle
 * between field-centric and robot-centric modes.
 *
 * Controls:
 * - Left Stick: Strafe (field-oriented when enabled, robot-oriented when
 * disabled)
 * - Right Stick X: Rotate
 * - Left Bumper: Toggle field-centric mode ON/OFF
 * - A Button (init): Reset to starting pose
 *
 * Field-Centric Mode:
 * - When enabled, pushing the left stick forward always moves the robot toward
 * the field's "forward" direction (same as autonomous start), regardless of
 * the robot's current heading
 * - The robot uses the saved autonomous end pose as the reference heading
 *
 * Robot-Centric Mode:
 * - When disabled, pushing the left stick forward moves the robot in its
 * current facing direction (traditional control)
 */
@TeleOp(name = "Field-Centric TeleOp", group = "TeleOp")
public class FieldCentricTeleOp extends OpMode {

    private Follower follower;

    // Drive motors
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    // Field-centric control
    private boolean fieldCentricEnabled = true;
    private double initialHeading = Math.toRadians(90); // Default: 90° (autonomous start)
    private boolean lastLeftBumper = false;

    // Default starting pose (if autonomous didn't run)
    private final Pose defaultStartPose = new Pose(110.5, 134.5, Math.toRadians(90));

    @Override
    public void init() {
        // === Initialize Pedro Follower ===
        follower = Constants.createFollower(hardwareMap);

        // Check if autonomous ran and saved a pose
        Pose startPose;
        if (GlobalData.hasAutonomousRun && GlobalData.autonomousEndPose != null) {
            startPose = GlobalData.autonomousEndPose;
            initialHeading = startPose.getHeading();
            telemetry.addLine("✅ Using autonomous end pose");
            telemetry.addData("Initial Heading", "%.1f°", Math.toDegrees(initialHeading));
        } else {
            startPose = defaultStartPose;
            initialHeading = defaultStartPose.getHeading();
            telemetry.addLine("⚠️ No autonomous data - using default pose");
            telemetry.addData("Default Heading", "%.1f°", Math.toDegrees(initialHeading));
        }

        // Set starting pose and update a few times
        follower.setStartingPose(startPose);
        for (int i = 0; i < 5; i++) {
            follower.update();
        }

        // === Initialize Drive Motors ===
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

        telemetry.addLine("=== INITIALIZATION COMPLETE ===");
        telemetry.addData("Field-Centric Mode", fieldCentricEnabled ? "ENABLED" : "DISABLED");
        telemetry.addLine("Press Left Bumper to toggle mode");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        follower.update();
        Pose currentPose = follower.getPose();

        // Manual reset with A button
        if (gamepad1.a) {
            Pose resetPose = GlobalData.hasAutonomousRun && GlobalData.autonomousEndPose != null
                    ? GlobalData.autonomousEndPose
                    : defaultStartPose;
            follower.setStartingPose(resetPose);
            telemetry.addLine("🔄 Pose reset!");
        }

        telemetry.addLine("=== READY TO START ===");
        telemetry.addData("Position", "X: %.1f, Y: %.1f", currentPose.getX(), currentPose.getY());
        telemetry.addData("Heading", "%.1f°", Math.toDegrees(currentPose.getHeading()));
        telemetry.addData("Field-Centric", fieldCentricEnabled ? "ENABLED" : "DISABLED");
        telemetry.addLine("Press 'A' to reset pose");
        telemetry.update();
    }

    @Override
    public void loop() {
        // === Update odometry ===
        follower.update();
        Pose robotPose = follower.getPose();

        // === Toggle field-centric mode ===
        boolean currentLeftBumper = gamepad1.left_bumper;
        if (currentLeftBumper && !lastLeftBumper) {
            fieldCentricEnabled = !fieldCentricEnabled;
        }
        lastLeftBumper = currentLeftBumper;

        // === Get joystick inputs ===
        double forwardSpeed = -gamepad1.left_stick_y; // Forward/backward
        double strafeSpeed = gamepad1.left_stick_x; // Left/right strafe
        double turnSpeed = gamepad1.right_stick_x; // Rotation

        // === Calculate motor powers ===
        double frontLeftPower, frontRightPower, backLeftPower, backRightPower;

        if (fieldCentricEnabled) {
            // Field-centric mode: rotate inputs by heading offset
            double robotHeading = robotPose.getHeading();
            double headingOffset = robotHeading - initialHeading;

            // Rotate the movement vector by negative heading offset
            double rotatedForward = forwardSpeed * Math.cos(-headingOffset) - strafeSpeed * Math.sin(-headingOffset);
            double rotatedStrafe = forwardSpeed * Math.sin(-headingOffset) + strafeSpeed * Math.cos(-headingOffset);

            // Calculate motor powers using rotated inputs
            frontLeftPower = rotatedForward + rotatedStrafe + turnSpeed;
            frontRightPower = rotatedForward - rotatedStrafe - turnSpeed;
            backLeftPower = rotatedForward - rotatedStrafe + turnSpeed;
            backRightPower = rotatedForward + rotatedStrafe - turnSpeed;
        } else {
            // Robot-centric mode: use raw inputs
            frontLeftPower = forwardSpeed + strafeSpeed + turnSpeed;
            frontRightPower = forwardSpeed - strafeSpeed - turnSpeed;
            backLeftPower = forwardSpeed - strafeSpeed + turnSpeed;
            backRightPower = forwardSpeed + strafeSpeed - turnSpeed;
        }

        // === Normalize powers ===
        double maxPower = Math.max(Math.abs(frontLeftPower),
                Math.max(Math.abs(frontRightPower),
                        Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))));

        if (maxPower > 1.0) {
            frontLeftPower /= maxPower;
            frontRightPower /= maxPower;
            backLeftPower /= maxPower;
            backRightPower /= maxPower;
        }

        // === Set motor powers ===
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);

        // === Display Telemetry ===
        telemetry.addLine("=== DRIVE MODE ===");
        telemetry.addData("Mode", fieldCentricEnabled ? "FIELD-CENTRIC" : "ROBOT-CENTRIC");
        telemetry.addData("Toggle", "Left Bumper");

        telemetry.addLine("=== ROBOT STATE ===");
        telemetry.addData("Position", "X: %.1f, Y: %.1f", robotPose.getX(), robotPose.getY());
        telemetry.addData("Current Heading", "%.1f°", Math.toDegrees(robotPose.getHeading()));
        telemetry.addData("Reference Heading", "%.1f°", Math.toDegrees(initialHeading));

        if (fieldCentricEnabled) {
            double headingOffset = robotPose.getHeading() - initialHeading;
            telemetry.addData("Heading Offset", "%.1f°", Math.toDegrees(headingOffset));
        }

        telemetry.addLine("=== CONTROLS ===");
        telemetry.addLine("Left Stick: Strafe");
        telemetry.addLine("Right Stick X: Rotate");
        telemetry.addLine("Left Bumper: Toggle Mode");

        telemetry.update();
    }

    @Override
    public void stop() {
        // Stop all motors
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
}
