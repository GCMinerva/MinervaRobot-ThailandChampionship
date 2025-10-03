package org.firstinspires.ftc.teamcode.test;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "Shooter Test", group = "Test")
public class ShooterTest extends OpMode {

    private RobotConfig robot;
    private Follower follower;

    private boolean lastAButton = false;
    private boolean lastBButton = false;

    @Override
    public void init() {
        // Initialize robot config
        robot = new RobotConfig(hardwareMap);

        // Initialize follower for position tracking
        follower = Constants.createFollower(hardwareMap);

        // Set target shooting position (example: blue alliance top left at 11, 135)
        robot.shooter.setTargetPosition(11.0, 135.0);

        // Set calibration values (example values - adjust based on testing)
        // minDistance, minPower, maxDistance, maxPower
        robot.shooter.setCalibration(20.0, 0.3, 60.0, 0.9);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Target", "X: 11, Y: 135");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Update follower to get current position
        follower.update();
        Pose currentPose = follower.getPose();

        // A button: Prepare and shoot
        boolean aButton = gamepad1.a;
        if (aButton && !lastAButton) {
            if (robot.shooter.isReadyToShoot()) {
                // Prepare shooter with power based on distance
                robot.shooter.prepareShot(currentPose);

                // Shoot after a brief delay (you might want to add a timer)
                robot.shooter.shoot();
            }
        }
        lastAButton = aButton;

        // B button: Reset servo for next ball
        boolean bButton = gamepad1.b;
        if (bButton && !lastBButton) {
            robot.shooter.reset();
        }
        lastBButton = bButton;

        // X button: Stop shooter
        if (gamepad1.x) {
            robot.shooter.stopAll();
        }

        // Display telemetry
        double distance = robot.shooter.getDistanceToTarget(currentPose);
        double calculatedPower = robot.shooter.getCalculatedPower(currentPose);
        boolean inRange = robot.shooter.isInRange(currentPose);

        telemetry.addData("Robot Position", "X: %.1f, Y: %.1f", currentPose.getX(), currentPose.getY());
        telemetry.addData("Target Position", "X: 11, Y: 135");
        telemetry.addData("Distance", "%.1f inches", distance);
        telemetry.addData("Calculated Power", "%.2f", calculatedPower);
        telemetry.addData("In Range", inRange ? "YES" : "NO");
        telemetry.addData("Ready to Shoot", robot.shooter.isReadyToShoot() ? "YES" : "NO");
        telemetry.addData("", "");
        telemetry.addData("Controls", "");
        telemetry.addData("A", "Prepare & Shoot");
        telemetry.addData("B", "Reset Servo");
        telemetry.addData("X", "Stop Shooter");
        telemetry.update();
    }
}
