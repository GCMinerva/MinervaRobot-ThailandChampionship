package org.firstinspires.ftc.teamcode.auto.test;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "AutoPath4", group = "Test")
public class AutoPath4 extends OpMode {

    private Follower follower;
    private Timer opmodeTimer;
    private PathChain chain; // single chain that contains all subpaths

    private final Pose startPose = new Pose(56.000, 8.000, Math.toRadians(90));

    private void buildPaths() {
        PathBuilder builder = new PathBuilder(follower, Constants.pathConstraints);

        chain = builder
                .addPath(
                        // Path 1
                        new BezierLine(new Pose(64.000, 80.000), new Pose(58.540, 85.868))
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(134))
                .addPath(
                        // Path 2
                        new BezierCurve(
                                new Pose(58.540, 85.868),
                                new Pose(54.053, 83.012),
                                new Pose(42.834, 83.216)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(134), Math.toRadians(180))
                .addPath(
                        // Path 3
                        new BezierLine(new Pose(42.834, 83.216), new Pose(21.622, 83.420))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .addPath(
                        // Path 4
                        new BezierCurve(
                                new Pose(21.622, 83.420),
                                new Pose(24.885, 71.590),
                                new Pose(15.707, 72.610)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .addPath(
                        // Path 5
                        new BezierLine(new Pose(15.707, 72.610), new Pose(58.540, 85.868))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(134))
                .addPath(
                        // Path 6
                        new BezierCurve(
                                new Pose(58.540, 85.868),
                                new Pose(69.758, 56.089),
                                new Pose(42.223, 59.556)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(134), Math.toRadians(180))
                .addPath(
                        // Path 7
                        new BezierLine(new Pose(42.223, 59.556), new Pose(22.030, 59.760))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .addPath(
                        // Path 8
                        new BezierLine(new Pose(22.030, 59.760), new Pose(58.540, 85.868))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(134))
                .addPath(
                        // Path 9
                        new BezierCurve(
                                new Pose(58.540, 85.868),
                                new Pose(70.166, 34.469),
                                new Pose(42.223, 35.284)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(134), Math.toRadians(180))
                .addPath(
                        // Path 10
                        new BezierLine(new Pose(42.223, 35.284), new Pose(21.826, 35.488))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .addPath(
                        // Path 11
                        new BezierLine(new Pose(21.826, 35.488), new Pose(62.823, 15.296))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(112))
                .build();
    }

    @Override
    public void init() {
        opmodeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        // Start following the whole chain. Use true to hold endpoint when done.
        follower.followPath(chain, true);
    }

    @Override
    public void loop() {
        // REQUIRED: updates controller and runs path following
        follower.update();

        // Telemetry
        Pose p = follower.getPose();
        telemetry.addData("busy", follower.isBusy());
        telemetry.addData("x", String.format("%.2f", p.getX()));
        telemetry.addData("y", String.format("%.2f", p.getY()));
        telemetry.addData("heading(deg)", String.format("%.1f", Math.toDegrees(p.getHeading())));
        telemetry.addData("elapsed(s)", String.format("%.2f", opmodeTimer.getElapsedTimeSeconds()));
        telemetry.update();
    }

    @Override
    public void stop() {
        // Nothing special; follower stops automatically when not busy.
    }
}
