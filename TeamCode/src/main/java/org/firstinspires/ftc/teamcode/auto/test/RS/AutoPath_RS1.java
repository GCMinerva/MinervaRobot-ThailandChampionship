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

@Autonomous(name = "AutoPath_RS1", group = "Test")
public class AutoPath_RS1 extends LinearOpMode {

    private Follower follower;

    @Override
    public void runOpMode() {

        // Create follower and set starting pose
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

        telemetry.addLine("✅ Paths ready — waiting for start");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // ✅ Run each path at its own speed
        followWithSpeed(path1, 1, "Path 1 — slow");
        followWithSpeed(path2, 1, "Path 2 — slower");
        followWithSpeed(path3, 0.25, "Path 3 — slowesttttttt veryyyyy slow");
        followWithSpeed(path4, 1, "Path 4 — slow");

        telemetry.addLine("✅ All paths finished");
        telemetry.update();
    }

    /** Helper to follow a path with a speed cap (Baron style) */
    private void followWithSpeed(PathChain path, double speed, String label) {
        telemetry.addLine("▶ " + label + " | Speed: " + speed);
        telemetry.update();

        // Run the path with that specific speed (sets local max power)
        follower.followPath(path, speed, true);

        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            Pose p = follower.getPose();
            telemetry.addData("Running", label);
            telemetry.addData("x", "%.2f", p.getX());
            telemetry.addData("y", "%.2f", p.getY());
            telemetry.addData("heading", "%.1f°", Math.toDegrees(p.getHeading()));
            telemetry.update();
        }

        telemetry.addLine("✔ Done: " + label);
        telemetry.update();
    }
}
