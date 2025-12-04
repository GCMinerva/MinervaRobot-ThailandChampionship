package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.archived.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.RobotHardwareConfig;

/**
 * Turret TeleOp Test
 *
 * Controls:
 * - Gamepad1 Left Stick: Strafe (forward/back/left/right)
 * - Gamepad1 Right Stick X: Rotate
 * - Gamepad1 D-Pad: Adjust goal position
 * - Gamepad1 A: Reset goal to starting position
 * - Gamepad1 Y: Reset turret to center (0°)
 *
 * Goal Position: Adjustable (default: 144, 144) - RED GOAL
 *
 * Telemetry shows:
 * - Robot position and heading (from odometry)
 * - Turret current/target angles
 * - Alignment status
 * - Encoder position (ticks)
 * - Current goal position
 */
@Disabled
@TeleOp(name = "Turret TeleOp Test", group = "Test")
public class TurretTeleOpTest extends LinearOpMode {

    private Follower follower;
    private TurretSubsystem turret;

    // Drive motors
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    // Goal position - RED GOAL
    private static final double GOAL_X = 144.0;
    private static final double GOAL_Y = 144.0;

    // Starting pose
    private final Pose startPose = new Pose(110.5, 134.5, Math.toRadians(90));

    // Manual angle control
    private double manualTargetAngle = 0.0;

    @Override
    public void runOpMode() {
        // === Initialize Pedro Follower ===
        follower = Constants.createFollower(hardwareMap);

        // Match FullTeleFixed pattern: Set pose, then update a few times
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

        // === Initialize Turret Subsystem (before waitForStart) ===
        turret = new TurretSubsystem(hardwareMap);
        turret.setGoalPosition(GOAL_X, GOAL_Y);
        turret.setStartingAngle(-90);  // Position turret to -90° during init

        // === Initialization Loop ===
        while (!isStarted() && !isStopRequested()) {
            follower.update();
            Pose currentPose = follower.getPose();

            // Auto-correction: If pose is zero (reset by driver), re-apply starting pose
            if (currentPose.getX() == 0 && currentPose.getY() == 0 && currentPose.getHeading() == 0) {
                follower.setStartingPose(startPose);
            }

            // Manual reset
            if (gamepad1.a) {
                follower.setStartingPose(startPose);
            }

            telemetry.addLine("=== INITIALIZATION ===");
            telemetry.addData("Position", "X: %.1f, Y: %.1f", currentPose.getX(), currentPose.getY());
            telemetry.addData("Heading", "%.1f°", Math.toDegrees(currentPose.getHeading()));
            telemetry.addData("Turret Angle", "%.1f°", turret.getCurrentAngle());
            telemetry.addLine("Press 'A' to force reset starting pose");
            telemetry.update();
        }

        while (opModeIsActive()) {
            // === Manual Mecanum Drive Control ===
            double forwardSpeed = -gamepad1.left_stick_y;  // Forward/backward
            double strafeSpeed = gamepad1.left_stick_x;    // Left/right strafe
            double turnSpeed = gamepad1.right_stick_x;     // Rotation

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

            // === Update odometry ===
            follower.update();
            Pose robotPose = follower.getPose();

            // === Handle Gamepad Input ===
            handleGamepadInput();

            // === Update turret toward blue goal ===
            turret.setGoalPosition(GOAL_X, GOAL_Y);
            turret.update(robotPose);

            // === Display Telemetry ===
            displayTelemetry(robotPose);
        }

        // === Cleanup ===
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        turret.stop();
        telemetry.addLine("Test Complete");
        telemetry.update();
    }

    /**
     * Handle gamepad input for goal position adjustment
     */
    private void handleGamepadInput() {
        // D-Pad Left/Right: Adjust goal X position
        if (gamepad1.dpad_left) {
            double goalX = turret.getGoalX() - 2.0;  // 2 inches per update
            turret.setGoalPosition(goalX, turret.getGoalY());
        }
        if (gamepad1.dpad_right) {
            double goalX = turret.getGoalX() + 2.0;  // 2 inches per update
            turret.setGoalPosition(goalX, turret.getGoalY());
        }

        // D-Pad Up/Down: Adjust goal Y position
        if (gamepad1.dpad_up) {
            double goalY = turret.getGoalY() + 2.0;  // 2 inches per update
            turret.setGoalPosition(turret.getGoalX(), goalY);
        }
        if (gamepad1.dpad_down) {
            double goalY = turret.getGoalY() - 2.0;  // 2 inches per update
            turret.setGoalPosition(turret.getGoalX(), goalY);
        }

        // A Button: Reset goal to original position
        if (gamepad1.a) {
            turret.setGoalPosition(GOAL_X, GOAL_Y);
            sleep(200);
        }

        // Y Button: Reset turret to center (0°)
        if (gamepad1.y) {
            turret.resetToCenter();
            sleep(200);
        }
    }

    /**
     * Display all relevant telemetry
     */
    private void displayTelemetry(Pose robotPose) {
        telemetry.addLine("=== ROBOT STATE ===");
        telemetry.addData("Robot X", "%.1f in", robotPose.getX());
        telemetry.addData("Robot Y", "%.1f in", robotPose.getY());
        telemetry.addData("Robot Heading", "%.1f°", Math.toDegrees(robotPose.getHeading()));

        telemetry.addLine("=== TURRET STATE ===");
        telemetry.addData("Current Angle", "%.1f°", turret.getCurrentAngle());
        telemetry.addData("Target Angle", "%.1f°", turret.getTargetAngle());
        telemetry.addData("Angle Error", "%.1f°", turret.getAngleError());
        telemetry.addData("Encoder Ticks", turret.turretMotor.getCurrentPosition());
        telemetry.addData("Aligned", turret.isAligned());

        telemetry.addLine("=== GOAL (ADJUSTABLE) ===");
        telemetry.addData("Goal X", "%.1f in", turret.getGoalX());
        telemetry.addData("Goal Y", "%.1f in", turret.getGoalY());
        telemetry.addData("Default Goal", "(%.1f, %.1f)", GOAL_X, GOAL_Y);

        telemetry.addLine("=== CONTROLS ===");
        telemetry.addLine("Left Stick: Strafe (forward/back/left/right)");
        telemetry.addLine("Right Stick X: Rotate");
        telemetry.addLine("D-Pad: Adjust goal position");
        telemetry.addLine("A: Reset goal | Y: Reset turret");

        telemetry.update();
    }
}