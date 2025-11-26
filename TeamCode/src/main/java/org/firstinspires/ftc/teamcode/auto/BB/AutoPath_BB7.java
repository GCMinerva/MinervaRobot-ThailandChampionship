package org.firstinspires.ftc.teamcode.auto.test.BB;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "AutoPath_BB7", group = "Test")
public class AutoPath_BB7 extends LinearOpMode {

    private Follower follower;

    @Override
    public void runOpMode() {
        follower = Constants.createFollower(hardwareMap);
//        follower.setStartingPose(new Pose(104.000, 136.000, Math.toRadians(90)).mirror());
        follower.setStartingPose(new Pose(110.5, 134.5, Math.toRadians(90)).mirror());

        // --- Build paths (fully mirrored) ---
        PathChain path1 = new PathBuilder(follower, Constants.pathConstraints)
                .addPath(new BezierLine(
                        new Pose(110.5, 134.5).mirror(),
                        new Pose(87.995, 85.144).mirror()))
                // 90° → 90°, 48° → 132°
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(132))
                .build();

        PathChain path2 = new PathBuilder(follower, Constants.pathConstraints)
                .addPath(new BezierCurve(
                        new Pose(87.995, 85.144).mirror(),
                        new Pose(92.923, 84.321).mirror(),
                        new Pose(102.512, 83.335).mirror()))
                // 48° → 132°, 0° → 180°
                .setLinearHeadingInterpolation(Math.toRadians(132), Math.toRadians(180))
                .build();

        PathChain path3 = new PathBuilder(follower, Constants.pathConstraints)
                .addPath(new BezierLine(
                        new Pose(102.512, 83.335).mirror(),
                        new Pose(120.971, 83.246).mirror()))
                // 0° → 180°, 0° → 180°
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        PathChain path4 = new PathBuilder(follower, Constants.pathConstraints)
                .addPath(new BezierLine(
                        new Pose(120.971, 83.246).mirror(),
                        new Pose(87.995, 85.144).mirror()))
                // 90° → 90°, 48° → 132°
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(132))
                .build();

        PathChain path5 = new PathBuilder(follower, Constants.pathConstraints)
                .addPath(new BezierCurve(
                        new Pose(87.995, 85.144).mirror(),
                        new Pose(88.174, 59.948).mirror(),
                        new Pose(102.332, 59.320).mirror()))
                // 48° → 132°, 0° → 180°
                .setLinearHeadingInterpolation(Math.toRadians(132), Math.toRadians(180))
                .build();

        PathChain path6 = new PathBuilder(follower, Constants.pathConstraints)
                .addPath(new BezierLine(
                        new Pose(102.332, 59.320).mirror(),
                        new Pose(120.971, 59.231).mirror()))
                // 0° → 180°, 0° → 180°
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        PathChain path7 = new PathBuilder(follower, Constants.pathConstraints)
                .addPath(new BezierLine(
                        new Pose(120.971, 59.231).mirror(),
                        new Pose(87.995, 85.144).mirror()))
                // 0° → 180°, 48° → 132°
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(132))
                .build();

        PathChain path8 = new PathBuilder(follower, Constants.pathConstraints)
                .addPath(new BezierCurve(
                        new Pose(87.995, 85.144).mirror(),
                        new Pose(75.450, 44.714).mirror(),
                        new Pose(101.795, 35.216).mirror()))
                // 48° → 132°, 0° → 180°
                .setLinearHeadingInterpolation(Math.toRadians(132), Math.toRadians(180))
                .build();

        PathChain path9 = new PathBuilder(follower, Constants.pathConstraints)
                .addPath(new BezierLine(
                        new Pose(101.795, 35.216).mirror(),
                        new Pose(120.254, 35.306).mirror()))
                // 0° → 180°, 0° → 180°
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        PathChain path10 = new PathBuilder(follower, Constants.pathConstraints)
                .addPath(new BezierLine(
                        new Pose(120.254, 35.306).mirror(),
                        new Pose(87.995, 85.144).mirror()))
                // 0° → 180°, 48° → 132°
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(132))
                .build();

        telemetry.addLine("✅ BB7 mirrored paths ready — waiting for start");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // --- Run all paths (default speed = 1.0) ---
        follow(path1, 1.0, "Path 1");
        follow(path2, 1.0, "Path 2");
        follow(path3, 0.25, "Path 3");
        follow(path4, 1.0, "Path 4");
        follow(path5, 1.0, "Path 5");
        follow(path6, 0.25, "Path 6");
        follow(path7, 1.0, "Path 7");
        follow(path8, 1.0, "Path 8");
        follow(path9, 0.25, "Path 9");
        follow(path10, 1.0, "Path 10");

        telemetry.addLine("✅ BB7 Auto Complete");
        telemetry.update();
    }

    // --- Follow helper ---
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
