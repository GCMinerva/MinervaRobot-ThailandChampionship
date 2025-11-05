package org.firstinspires.ftc.teamcode.auto.test.RS;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "AutoPath_RS6", group = "Test")
public class AutoPath_RS6 extends LinearOpMode {

    private Follower follower;

    @Override
    public void runOpMode() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(88.000, 8.000, Math.toRadians(90)));

        // Build each path segment
        PathChain path1 = new PathBuilder(follower, Constants.pathConstraints)
                .addPath(new BezierLine(new Pose(88.000, 8.000), new Pose(83.963, 16.309)))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(66))
                .build();

        PathChain path2 = new PathBuilder(follower, Constants.pathConstraints)
                .addPath(new BezierCurve(
                        new Pose(83.963, 16.309),
                        new Pose(87.278, 59.589),
                        new Pose(101.884, 59.679)))
                .setLinearHeadingInterpolation(Math.toRadians(66), Math.toRadians(0))
                .build();

        PathChain path3 = new PathBuilder(follower, Constants.pathConstraints)
                .addPath(new BezierLine(new Pose(101.884, 59.679), new Pose(121.240, 59.589)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        PathChain path4 = new PathBuilder(follower, Constants.pathConstraints)
                .addPath(new BezierCurve(
                        new Pose(121.240, 59.589),
                        new Pose(100.719, 59.052),
                        new Pose(96.956, 64.697),
                        new Pose(127.064, 63.711)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                .build();

        PathChain path5 = new PathBuilder(follower, Constants.pathConstraints)
                .addPath(new BezierCurve(
                        new Pose(127.064, 63.711),
                        new Pose(82.439, 66.579),
                        new Pose(83.963, 16.309)))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(66))
                .build();

        PathChain path6 = new PathBuilder(follower, Constants.pathConstraints)
                .addPath(new BezierCurve(
                        new Pose(83.963, 16.309),
                        new Pose(86.382, 31.990),
                        new Pose(102.332, 35.306)))
                .setLinearHeadingInterpolation(Math.toRadians(66), Math.toRadians(0))
                .build();

        PathChain path7 = new PathBuilder(follower, Constants.pathConstraints)
                .addPath(new BezierLine(new Pose(102.332, 35.306), new Pose(122.046, 35.754)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        PathChain path8 = new PathBuilder(follower, Constants.pathConstraints)
                .addPath(new BezierCurve(
                        new Pose(122.046, 35.754),
                        new Pose(121.598, 11.559),
                        new Pose(83.963, 16.309)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(66))
                .build();

        telemetry.addLine("✅ RS6 paths ready — waiting for start");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // Apply custom per-path speeds
        follow(path1, 1, "Path 1");
        follow(path2, 1, "Path 2");
        follow(path3, 0.25, "Path 3");
        follow(path4, 0.3, "Path 4");
        follow(path5, 1, "Path 5");
        follow(path6, 1, "Path 6");
        follow(path7, 0.25, "Path 7");
        follow(path8, 1, "Path 8");

        telemetry.addLine("✅ RS6 Auto Complete");
        telemetry.update();
    }

    private void follow(PathChain path, double speed, String label) {
        telemetry.addLine("▶ Running " + label + " | Speed: " + speed);
        telemetry.update();

        follower.followPath(path, speed, true);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            Pose p = follower.getPose();
            telemetry.addData("Path", label);
            telemetry.addData("Speed", speed);
            telemetry.addData("x", "%.2f", p.getX());
            telemetry.addData("y", "%.2f", p.getY());
            telemetry.addData("heading", "%.1f°", Math.toDegrees(p.getHeading()));
            telemetry.update();
        }

        telemetry.addLine("✔ Done " + label);
        telemetry.update();
    }
}
