package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.firstinspires.ftc.teamcode.RobotHardwareConfig;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * Minerva TeleOp - Mecanum Drive with Shooter Control
 *
 * Controls:
 * - Left Stick: Strafe (x/y movement)
 * - Right Stick X: Rotation
 * - A Button: Start/Set Shooter Motors
 * - B Button: Toggle Servo & Outtake (Shoot)
 * - X Button: Stop All Motors
 * - D-Pad Up/Down: Adjust Shooter Power
 *
 * @author MinervaRobot Team
 * @version 1.0
 */
@TeleOp(name = "Minerva TeleOp", group = "TeleOp")
public class MinervaTeleOp extends OpMode {

    private RobotConfig robot;
    private Follower follower;

    // Shooter control variables - using config constants
    private double shooterPower = (RobotHardwareConfig.ShooterParams.MIN_POWER + RobotHardwareConfig.ShooterParams.MAX_POWER) / 2;  // Start at mid-range
    private static final double POWER_INCREMENT = RobotHardwareConfig.ShooterParams.POWER_INCREMENT;
    private boolean servoExtended = false;
    private boolean motorsRunning = false;

    // Button state tracking for single press detection
    private boolean lastA = false;
    private boolean lastB = false;
    private boolean lastX = false;
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;

    @Override
    public void init() {
        // Initialize robot configuration
        robot = new RobotConfig(hardwareMap);

        // Initialize Pedro Pathing follower for mecanum drive
        follower = Constants.createFollower(hardwareMap);

        // Set starting pose (can be adjusted)
        follower.setStartingPose(new Pose(0, 0, 0));

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Mode", "TeleOp Ready");
        telemetry.update();
    }

    @Override
    public void loop() {
        // === MECANUM DRIVE CONTROL ===
        // Get gamepad inputs
        double forwardSpeed = -gamepad1.left_stick_y;  // Forward/backward
        double strafeSpeed = -gamepad1.left_stick_x;   // Left/right strafe
        double turnSpeed = -gamepad1.right_stick_x;    // Rotation

        // Set drive powers using zero power behavior (brake when sticks centered)
        follower.setTeleOpDrive(forwardSpeed, strafeSpeed, turnSpeed, true);
        follower.update();

        // === SHOOTER POWER ADJUSTMENT ===
        // D-Pad Up: Increase shooter power (and update motors if running)
        if (gamepad1.dpad_up && !lastDpadUp) {
            shooterPower = Math.min(1.0, shooterPower + POWER_INCREMENT);
            if (motorsRunning) {
                robot.shooter.setShooterPower(shooterPower);
            }
        }
        lastDpadUp = gamepad1.dpad_up;

        // D-Pad Down: Decrease shooter power (and update motors if running)
        if (gamepad1.dpad_down && !lastDpadDown) {
            shooterPower = Math.max(0.0, shooterPower - POWER_INCREMENT);
            if (motorsRunning) {
                robot.shooter.setShooterPower(shooterPower);
            }
        }
        lastDpadDown = gamepad1.dpad_down;

        // === SHOOTER CONTROLS ===
        // A Button: Set shooter motor power
        boolean aButton = gamepad1.a;
        if (aButton && !lastA) {
            robot.shooter.setShooterPower(shooterPower);
            motorsRunning = true;
        }
        lastA = aButton;

        // B Button: Toggle servo and outtake (shoot/reset)
        boolean bButton = gamepad1.b;
        if (bButton && !lastB) {
            if (!servoExtended) {
                // Shoot: extend servo and activate outtake
                robot.shooter.setServoPosition(RobotHardwareConfig.ShooterServos.SHOOT_POSITION);
                robot.shooter.feedBall();
                servoExtended = true;
            } else {
                // Reset: retract servo and stop outtake
                robot.shooter.setServoPosition(RobotHardwareConfig.ShooterServos.RESET_POSITION);
                robot.shooter.stopFeed();
                servoExtended = false;
            }
        }
        lastB = bButton;

        // X Button: Stop all motors
        boolean xButton = gamepad1.x;
        if (xButton && !lastX) {
            robot.shooter.stopAll();
            servoExtended = false;
            motorsRunning = false;
        }
        lastX = xButton;

        // === TELEMETRY ===
        Pose currentPose = follower.getPose();

        telemetry.addData("=== DRIVE ===", "");
        telemetry.addData("Position", "X: %.1f, Y: %.1f", currentPose.getX(), currentPose.getY());
        telemetry.addData("Heading", "%.1f°", Math.toDegrees(currentPose.getHeading()));
        telemetry.addData("", "");

        telemetry.addData("=== SHOOTER ===", "");
        telemetry.addData("Power", "%.2f", shooterPower);
        telemetry.addData("Motors", motorsRunning ? "RUNNING" : "STOPPED");
        telemetry.addData("Servo", servoExtended ? String.format("SHOOT (%.1f)", RobotHardwareConfig.ShooterServos.SHOOT_POSITION) : String.format("RESET (%.1f)", RobotHardwareConfig.ShooterServos.RESET_POSITION));
        telemetry.addData("Outtake", servoExtended ? "ON" : "OFF");
        telemetry.addData("", "");

        telemetry.addData("=== CONTROLS ===", "");
        telemetry.addData("Left Stick", "Strafe");
        telemetry.addData("Right Stick X", "Rotate");
        telemetry.addData("D-Pad Up/Down", "Adjust Power (Live)");
        telemetry.addData("A", "Start Motors");
        telemetry.addData("B", "Toggle Servo & Outtake");
        telemetry.addData("X", "Stop All Motors");

        telemetry.update();
    }

    @Override
    public void stop() {
        robot.stopAllMotors();
    }
}
