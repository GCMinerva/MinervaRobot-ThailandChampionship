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

@Autonomous(name = "AutoPath_BS9", group = "Test")
public class AutoPath_BS9 extends LinearOpMode {

    private Follower follower;

    @Override
    public void runOpMode() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(88.000, 8.000, Math.toRadians(90)).mirror());

        // --- Build all paths (mirrored) ---
        PathChain path1 = new PathBuilder(follower, Constants.pathConstraints)
                .addPath(new BezierLine(
                        new Pose(88.000, 8.000).mirror(),
                        new Pose(83.963, 16.309).mirror()))
                // 90° → 90°, 66° → 114°
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(114))
                .build();

        PathChain path2 = new PathBuilder(follower, Constants.pathConstraints)
                .addPath(new BezierCurve(
                        new Pose(83.963, 16.309).mirror(),
                        new Pose(87.637, 38.173).mirror(),
                        new Pose(102.243, 35.395).mirror()))
                // 66° → 114°, 0° → 180°
                .setLinearHeadingInterpolation(Math.toRadians(114), Math.toRadians(180))
                .build();

        PathChain path3 = new PathBuilder(follower, Constants.pathConstraints)
                .addPath(new BezierLine(
                        new Pose(102.243, 35.395).mirror(),
                        new Pose(121.598, 35.574).mirror()))
                // 0° → 180°, 0° → 180°
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        PathChain path4 = new PathBuilder(follower, Constants.pathConstraints)
                .addPath(new BezierCurve(
                        new Pose(121.598, 35.574).mirror(),
                        new Pose(119.895, 15.502).mirror(),
                        new Pose(83.963, 16.309).mirror()))
                // 0° → 180°, 66° → 114°
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(114))
                .build();

        PathChain path5 = new PathBuilder(follower, Constants.pathConstraints)
                .addPath(new BezierCurve(
                        new Pose(83.963, 16.309).mirror(),
                        new Pose(71.418, 13.083).mirror(),
                        new Pose(34.947, 25.717).mirror(),
                        new Pose(17.026, 11.559).mirror()))
                // 66° → 114°, 0° → 180°
                .setLinearHeadingInterpolation(Math.toRadians(114), Math.toRadians(180))
                .build();

        telemetry.addLine("✅ Mirrored BS9 paths ready — waiting for start");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // --- Run each path with custom speeds ---
        follow(path1, 1, "Path 1");
        follow(path2, 1, "Path 2");
        follow(path3, 0.25, "Path 3");
        follow(path4, 1, "Path 4");
        follow(path5, 1, "Path 5");

        telemetry.addLine("✅ BS9 Auto Complete");
        telemetry.update();
    }

    // --- Follow helper function ---
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
