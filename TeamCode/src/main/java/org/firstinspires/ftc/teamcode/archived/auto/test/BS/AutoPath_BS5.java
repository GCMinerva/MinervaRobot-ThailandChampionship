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

@Autonomous(name = "AutoPath_BS5", group = "Test")
public class AutoPath_BS5 extends LinearOpMode {

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
                        new Pose(87.278, 59.589).mirror(),
                        new Pose(101.884, 59.679).mirror()))
                // 66° → 114°, 0° → 180°
                .setLinearHeadingInterpolation(Math.toRadians(114), Math.toRadians(180))
                .build();

        PathChain path6 = new PathBuilder(follower, Constants.pathConstraints)
                .addPath(new BezierLine(
                        new Pose(101.884, 59.679).mirror(),
                        new Pose(121.240, 59.589).mirror()))
                // 0° → 180°, 0° → 180°
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        PathChain path7 = new PathBuilder(follower, Constants.pathConstraints)
                .addPath(new BezierCurve(
                        new Pose(121.240, 59.589).mirror(),
                        new Pose(91.042, 65.862).mirror(),
                        new Pose(87.995, 85.144).mirror()))
                // 0° → 180°, 48° → 132°
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(132))
                .build();

        PathChain path8 = new PathBuilder(follower, Constants.pathConstraints)
                .addPath(new BezierLine(
                        new Pose(87.995, 85.144).mirror(),
                        new Pose(102.243, 83.425).mirror()))
                // 48° → 132°, 0° → 180°
                .setLinearHeadingInterpolation(Math.toRadians(132), Math.toRadians(180))
                .build();

        PathChain path9 = new PathBuilder(follower, Constants.pathConstraints)
                .addPath(new BezierLine(
                        new Pose(102.243, 83.425).mirror(),
                        new Pose(121.867, 83.604).mirror()))
                // 0° → 180°, 0° → 180°
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        PathChain path10 = new PathBuilder(follower, Constants.pathConstraints)
                .addPath(new BezierLine(
                        new Pose(121.867, 83.604).mirror(),
                        new Pose(87.995, 85.144).mirror()))
                // 0° → 180°, 48° → 132°
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(132))
                .build();

        telemetry.addLine("✅ Mirrored BS5 ready — waiting for start");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // Custom speeds
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

        telemetry.addLine("✅ BS5 Auto Complete");
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
