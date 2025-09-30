package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Auto Test", group = "Test")
public class AutoTest extends OpMode {

    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState;

    private final Pose startPose = new Pose(56.000, 8.000, Math.toRadians(90));

    private PathChain line1, path2, path3;

    public void buildPaths() {
        PathBuilder builder = new PathBuilder(follower, Constants.pathConstraints);

        line1 = builder
                .addPath(
                        new BezierCurve(
                                new Pose(55.728, 11.486),
                                new Pose(8.083, 90.399),
                                new Pose(90.186, 86.357),
                                new Pose(89.123, 110.180)
                        ))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(45))
                .build();

//        path2 = builder
//                .addPath(
//                        new BezierCurve(
//                                new Pose(95.716, 71.256),
//                                new Pose(99.545, 118.688),
//                                new Pose(67.001, 118.050)
//                        )
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(270))
//                .build();
//
//        path3 = builder
//                .addPath(new BezierLine(new Pose(67.001, 118.050), new Pose(56.579, 8.083)))
//                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(0))
//                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(line1);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(path2, true);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(path3, true);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    setPathState(-1);
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {
//        follower.update();
//        autonomousPathUpdate();

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("elapsed time", opmodeTimer.getElapsedTimeSeconds());
        telemetry.update();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {
        follower.update();
        autonomousPathUpdate();
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void stop() {}
}