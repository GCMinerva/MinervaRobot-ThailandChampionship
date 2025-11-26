package org.firstinspires.ftc.teamcode.auto.test.BS;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "AutoPath_BS7", group = "Test")
public class AutoPath_BS7 extends LinearOpMode {

    private Follower follower;

    @Override
    public void runOpMode() {
        follower = Constants.createFollower(hardwareMap);
//        follower.setStartingPose(new Pose(88.000, 8.000, Math.toRadians(90)).mirror());
        follower.setStartingPose(new Pose(88.000, 9.3, Math.toRadians(90)).mirror());

        // === Path 1 ===
        PathChain path1 = new PathBuilder(follower, Constants.pathConstraints)
                .addPath(new BezierLine(
                        new Pose(88.000, 9.3).mirror(),
                        new Pose(83.963, 16.309).mirror()))
                // 90° → 90°, 66° → 114°
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(114))
                .build();

        // === Path 2 ===
        PathChain path2 = new PathBuilder(follower, Constants.pathConstraints)
                .addPath(new BezierCurve(
                        new Pose(83.963, 16.309).mirror(),
                        new Pose(87.278, 59.589).mirror(),
                        new Pose(101.884, 59.679).mirror()))
                // 66° → 114°, 0° → 180°
                .setLinearHeadingInterpolation(Math.toRadians(114), Math.toRadians(180))
                .build();

        // === Path 3 ===
        PathChain path3 = new PathBuilder(follower, Constants.pathConstraints)
                .addPath(new BezierLine(
                        new Pose(101.884, 59.679).mirror(),
                        new Pose(121.240, 59.589).mirror()))
                // 0° → 180°, 0° → 180°
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        // === Path 4 ===
        PathChain path4 = new PathBuilder(follower, Constants.pathConstraints)
                .addPath(new BezierCurve(
                        new Pose(121.240, 59.589).mirror(),
                        new Pose(100.719, 59.052).mirror(),
                        new Pose(96.956, 64.697).mirror(),
                        new Pose(127.064, 63.711).mirror()))
                // 0° → 180°, 90° → 90°
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))
                .build();

        // === Path 5 ===
        PathChain path5 = new PathBuilder(follower, Constants.pathConstraints)
                .addPath(new BezierCurve(
                        new Pose(127.064, 63.711).mirror(),
                        new Pose(82.439, 66.579).mirror(),
                        new Pose(83.963, 16.309).mirror()))
                // 90° → 90°, 66° → 114°
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(114))
                .build();

        // === Path 6 ===
        PathChain path6 = new PathBuilder(follower, Constants.pathConstraints)
                .addPath(new BezierCurve(
                        new Pose(83.963, 16.309).mirror(),
                        new Pose(86.382, 31.990).mirror(),
                        new Pose(102.332, 35.306).mirror()))
                // 66° → 114°, 0° → 180°
                .setLinearHeadingInterpolation(Math.toRadians(114), Math.toRadians(180))
                .build();

        // === Path 7 ===
        PathChain path7 = new PathBuilder(follower, Constants.pathConstraints)
                .addPath(new BezierLine(
                        new Pose(102.332, 35.306).mirror(),
                        new Pose(122.046, 35.754).mirror()))
                // 0° → 180°, 0° → 180°
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        // === Path 8 ===
        PathChain path8 = new PathBuilder(follower, Constants.pathConstraints)
                .addPath(new BezierCurve(
                        new Pose(122.046, 35.754).mirror(),
                        new Pose(121.598, 11.559).mirror(),
                        new Pose(83.963, 16.309).mirror()))
                // 0° → 180°, 66° → 114°
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(114))
                .build();

        // === Path 9 ===
        PathChain path9 = new PathBuilder(follower, Constants.pathConstraints)
                .addPath(new BezierCurve(
                        new Pose(83.963, 16.309).mirror(),
                        new Pose(88.000, 81.000).mirror(),
                        new Pose(102.153, 83.783).mirror()))
                // 66° → 114°, 0° → 180°
                .setLinearHeadingInterpolation(Math.toRadians(114), Math.toRadians(180))
                .build();

        // === Path 10 ===
        PathChain path10 = new PathBuilder(follower, Constants.pathConstraints)
                .addPath(new BezierLine(
                        new Pose(102.153, 83.783).mirror(),
                        new Pose(121.329, 83.963).mirror()))
                // 0° → 180°, 0° → 180°
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        // === Path 11 ===
        PathChain path11 = new PathBuilder(follower, Constants.pathConstraints)
                .addPath(new BezierLine(
                        new Pose(121.329, 83.963).mirror(),
                        new Pose(87.995, 85.144).mirror()))
                // 0° → 180°, 48° → 132°
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(132))
                .build();

        telemetry.addLine("✅ Mirrored BS7 paths ready — waiting for start");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // === Follow paths with different speeds ===
        followWithSpeed(path1, 1, "Path 1 - slow");
        followWithSpeed(path2, 1, "Path 2 - slower");
        followWithSpeed(path3, 0.3, "Path 3 - slowestttt");
        followWithSpeed(path4, 0.45, "Path 4 - slow");
        followWithSpeed(path5, 1, "Path 5 - slower");
        followWithSpeed(path6, 1, "Path 6 - slowest");
        followWithSpeed(path7, 0.22, "Path 7 - slow");
        followWithSpeed(path8, 1, "Path 8 - slow");
        followWithSpeed(path9, 1, "Path 9 - slower");
        followWithSpeed(path10, 0.23, "Path 10 - slowest");
        followWithSpeed(path11, 1, "Path 11 - slow");

        telemetry.addLine("✅ All BS7 paths complete");
        telemetry.update();
    }

    /** Helper function to follow each path with speed and telemetry */
    private void followWithSpeed(PathChain path, double speed, String label) {
        telemetry.addLine("▶ " + label + " | Speed: " + speed);
        telemetry.update();

        follower.followPath(path, speed, true);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            Pose p = follower.getPose();
            telemetry.addData("Running", label);
            telemetry.addData("x", "%.2f", p.getX());
            telemetry.addData("y", "%.2f", p.getY());
            telemetry.addData("heading", "%.1f°", Math.toDegrees(p.getHeading()));
            telemetry.addData("Speed", speed);
            telemetry.update();
        }

        telemetry.addLine("✔ Done " + label);
        telemetry.update();
    }
}
