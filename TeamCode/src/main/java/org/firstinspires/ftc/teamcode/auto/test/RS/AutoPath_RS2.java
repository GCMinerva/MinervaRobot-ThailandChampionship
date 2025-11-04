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

@Autonomous(name = "AutoPath_RS2", group = "Test")
public class AutoPath_RS2 extends LinearOpMode {

    private Follower follower;

    @Override
    public void runOpMode() {

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(88.000, 8.000, Math.toRadians(90)));

        // === Path 1 ===
        PathChain path1 = new PathBuilder(follower, Constants.pathConstraints)
                .addPath(new BezierLine(
                        new Pose(88.000, 8.000),
                        new Pose(83.963, 16.309)))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(66))
                .build();

        // === Path 2 ===
        PathChain path2 = new PathBuilder(follower, Constants.pathConstraints)
                .addPath(new BezierCurve(
                        new Pose(83.963, 16.309),
                        new Pose(87.637, 38.173),
                        new Pose(102.243, 35.395)))
                .setLinearHeadingInterpolation(Math.toRadians(66), Math.toRadians(0))
                .build();

        // === Path 3 ===
        PathChain path3 = new PathBuilder(follower, Constants.pathConstraints)
                .addPath(new BezierLine(
                        new Pose(102.243, 35.395),
                        new Pose(121.598, 35.574)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        // === Path 4 ===
        PathChain path4 = new PathBuilder(follower, Constants.pathConstraints)
                .addPath(new BezierCurve(
                        new Pose(121.598, 35.574),
                        new Pose(119.895, 15.502),
                        new Pose(83.963, 16.309)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(66))
                .build();

        // === Path 5 ===
        PathChain path5 = new PathBuilder(follower, Constants.pathConstraints)
                .addPath(new BezierCurve(
                        new Pose(83.963, 16.309),
                        new Pose(87.278, 59.589),
                        new Pose(101.884, 59.679)))
                .setLinearHeadingInterpolation(Math.toRadians(66), Math.toRadians(0))
                .build();

        // === Path 6 ===
        PathChain path6 = new PathBuilder(follower, Constants.pathConstraints)
                .addPath(new BezierLine(
                        new Pose(101.884, 59.679),
                        new Pose(121.240, 59.589)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        // === Path 7 ===
        PathChain path7 = new PathBuilder(follower, Constants.pathConstraints)
                .addPath(new BezierLine(
                        new Pose(121.240, 59.589),
                        new Pose(83.963, 16.309)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(66))
                .build();

        telemetry.addLine("✅ RS2 paths ready — waiting for start");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // === Run all paths at default speed (1.0) ===
        followWithSpeed(path1, 1.0, "Path 1");
        followWithSpeed(path2, 1.0, "Path 2");
        followWithSpeed(path3, 1.0, "Path 3");
        followWithSpeed(path4, 1.0, "Path 4");
        followWithSpeed(path5, 1.0, "Path 5");
        followWithSpeed(path6, 1.0, "Path 6");
        followWithSpeed(path7, 1.0, "Path 7");

        telemetry.addLine("✅ RS2 Auto Complete — Full Speed");
        telemetry.update();
    }

    /** Helper to follow each path with speed and telemetry */
    private void followWithSpeed(PathChain path, double speed, String label) {
        telemetry.addLine("▶ Running " + label + " | Speed: " + speed);
        telemetry.update();

        follower.followPath(path, speed, true);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            Pose p = follower.getPose();
            telemetry.addData("Path", label);
            telemetry.addData("x", "%.2f", p.getX());
            telemetry.addData("y", "%.2f", p.getY());
            telemetry.addData("heading", "%.1f°", Math.toDegrees(p.getHeading()));
            telemetry.update();
        }

        telemetry.addLine("✔ Done: " + label);
        telemetry.update();
    }
}
