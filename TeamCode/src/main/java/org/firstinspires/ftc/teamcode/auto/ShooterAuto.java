package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Shooter Auto", group = "Auto")
public class ShooterAuto extends OpMode {

    private Follower follower;
    private RobotConfig robot;
    private Timer pathTimer, actionTimer;
    private int pathState;

    // Shooting target position (blue alliance top left)
    private final double TARGET_X = 11.0;
    private final double TARGET_Y = 135.0;

    // Starting position
    private final Pose startPose = new Pose(56.0, 8.0, Math.toRadians(90));

    @Override
    public void init() {
        pathTimer = new Timer();
        actionTimer = new Timer();

        // Initialize robot and follower
        robot = new RobotConfig(hardwareMap);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        // Configure shooter
        robot.shooter.setTargetPosition(TARGET_X, TARGET_Y);
        // Calibration: minDistance, minPower, maxDistance, maxPower
        robot.shooter.setCalibration(20.0, 0.3, 60.0, 0.9);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        pathState = 0;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {
        follower.update();
        Pose currentPose = follower.getPose();

        switch (pathState) {
            case 0:
                // Initial state - prepare shooter
                robot.shooter.prepareShot(currentPose);
                setPathState(1);
                break;

            case 1:
                // Wait for shooter to spin up (0.5 seconds)
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    robot.shooter.shoot();
                    actionTimer.resetTimer();
                    setPathState(2);
                }
                break;

            case 2:
                // Wait for ball to shoot (0.3 seconds)
                if (actionTimer.getElapsedTimeSeconds() > 0.3) {
                    robot.shooter.reset();
                    setPathState(3);
                }
                break;

            case 3:
                // Wait for servo to reset (0.5 seconds)
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    // Ready for next shot or continue path
                    setPathState(4);
                }
                break;

            case 4:
                // Check if in range for another shot
                if (robot.shooter.isInRange(currentPose)) {
                    // Prepare for another shot
                    robot.shooter.prepareShot(currentPose);
                    setPathState(5);
                } else {
                    // Out of range, end auto
                    robot.shooter.stop();
                    setPathState(-1);
                }
                break;

            case 5:
                // Second shot - wait for spin up
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    robot.shooter.shoot();
                    actionTimer.resetTimer();
                    setPathState(6);
                }
                break;

            case 6:
                // Second shot - wait for ball to shoot
                if (actionTimer.getElapsedTimeSeconds() > 0.3) {
                    robot.shooter.stop();
                    setPathState(-1);  // Done
                }
                break;

            default:
                // Autonomous complete
                robot.shooter.stop();
                break;
        }

        // Telemetry
        double distance = robot.shooter.getDistanceToTarget(currentPose);
        double power = robot.shooter.getCalculatedPower(currentPose);

        telemetry.addData("State", pathState);
        telemetry.addData("Position", "X: %.1f, Y: %.1f", currentPose.getX(), currentPose.getY());
        telemetry.addData("Distance to Target", "%.1f inches", distance);
        telemetry.addData("Shooter Power", "%.2f", power);
        telemetry.addData("In Range", robot.shooter.isInRange(currentPose) ? "YES" : "NO");
        telemetry.addData("Ready", robot.shooter.isReadyToShoot() ? "YES" : "NO");
        telemetry.update();
    }

    private void setPathState(int state) {
        pathState = state;
        pathTimer.resetTimer();
    }

    @Override
    public void stop() {
        robot.shooter.stop();
    }
}
