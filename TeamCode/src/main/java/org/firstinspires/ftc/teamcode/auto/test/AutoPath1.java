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
public class AutoPath1 extends OpMode {

    private Follower follower;
    private Timer opmodeTimer;
    private PathChain chain; // single chain that contains all subpaths

    private final Pose startPose = new Pose(56.000, 8.000, Math.toRadians(90));

    private void buildPaths() {
        PathBuilder builder = new PathBuilder(follower, Constants.pathConstraints);

        chain = builder
                .addPath(
                        // Path 1
                        new BezierLine(new Pose(56.000, 8.000), new Pose(64.047, 15.092))
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(116))
                .addPath(
                        // Path 2
                        new BezierCurve(
                                new Pose(64.047, 15.092),
                                new Pose(67.718, 36.508),
                                new Pose(40.999, 35.692)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(116), Math.toRadians(180))
                .addPath(
                        // Path 3
                        new BezierLine(new Pose(40.999, 35.692), new Pose(19.582, 35.488))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .addPath(
                        // Path 4
                        new BezierCurve(
                                new Pose(19.582, 35.488),
                                new Pose(15.503, 10.401),
                                new Pose(64.047, 15.092)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(116))
                .addPath(
                        // Path 5
                        new BezierCurve(
                                new Pose(64.047, 15.092),
                                new Pose(69.350, 60.984),
                                new Pose(40.795, 59.556)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(116), Math.toRadians(180))
                .addPath(
                        // Path 6
                        new BezierLine(new Pose(40.795, 59.556), new Pose(19.786, 59.352))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .addPath(
                        // Path 7
                        new BezierCurve(
                                new Pose(19.786, 59.352),
                                new Pose(45.894, 27.942),
                                new Pose(67.310, 68.531)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(131))
                .addPath(
                        // Path 8
                        new BezierCurve(
                                new Pose(67.310, 68.531),
                                new Pose(63.435, 63.636),
                                new Pose(40.591, 84.032)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(131), Math.toRadians(180))
                .addPath(
                        // Path 9
                        new BezierLine(new Pose(40.591, 84.032), new Pose(20.194, 83.828))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .addPath(
                        // Path 10
                        new BezierCurve(
                                new Pose(20.194, 83.828),
                                new Pose(34.472, 61.188),
                                new Pose(67.310, 68.531)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(131))
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
