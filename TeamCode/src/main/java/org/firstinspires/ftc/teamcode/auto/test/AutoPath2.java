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

@Autonomous(name = "AutoPath2", group = "Test")
public class AutoPath2 extends OpMode {

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
                                new Pose(73.837, 54.049),
                                new Pose(41.815, 59.964)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(116), Math.toRadians(180))
                .addPath(
                        // Path 3
                        new BezierLine(new Pose(41.815, 59.964), new Pose(18.766, 59.556))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .addPath(
                        // Path 4
                        new BezierCurve(
                                new Pose(18.766, 59.556),
                                new Pose(18.766, 65.471),
                                new Pose(15.503, 64.248)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .addPath(
                        // Path 5
                        new BezierCurve(
                                new Pose(15.503, 64.248),
                                new Pose(78.325, 62.820),
                                new Pose(64.251, 15.296)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(116))
                .addPath(
                        // Path 6
                        new BezierCurve(
                                new Pose(64.251, 15.296),
                                new Pose(73.633, 38.956),
                                new Pose(43.038, 35.284)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(116), Math.toRadians(180))
                .addPath(
                        // Path 7
                        new BezierLine(new Pose(43.038, 35.284), new Pose(21.214, 35.284))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .addPath(
                        // Path 8
                        new BezierCurve(
                                new Pose(21.214, 35.284),
                                new Pose(22.234, 12.236),
                                new Pose(64.251, 14.888)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(116))
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
