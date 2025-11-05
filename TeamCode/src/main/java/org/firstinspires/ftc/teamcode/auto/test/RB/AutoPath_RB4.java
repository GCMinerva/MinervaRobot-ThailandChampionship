package org.firstinspires.ftc.teamcode.auto.test.RB;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "AutoPath_RB4", group = "Test")
public class AutoPath_RB4 extends LinearOpMode {

    private Follower follower;

    @Override
    public void runOpMode() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(104.000, 136.000, Math.toRadians(90)));

        // --- Build paths ---
        PathChain path1 = new PathBuilder(follower, Constants.pathConstraints)
                .addPath(new BezierLine(
                        new Pose(104.000, 136.000),
                        new Pose(88.533, 85.934)))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(48))
                .build();

        PathChain path2 = new PathBuilder(follower, Constants.pathConstraints)
                .addPath(new BezierCurve(
                        new Pose(88.533, 85.934),
                        new Pose(84.948, 74.285),
                        new Pose(99.465, 71.955)))
                .setLinearHeadingInterpolation(Math.toRadians(48), Math.toRadians(90))
                .build();

        PathChain path3 = new PathBuilder(follower, Constants.pathConstraints)
                .addPath(new BezierLine(
                        new Pose(99.465, 71.955),
                        new Pose(127.423, 71.866)))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
                .build();

        PathChain path4 = new PathBuilder(follower, Constants.pathConstraints)
                .addPath(new BezierLine(
                        new Pose(127.423, 71.866),
                        new Pose(89.608, 72.134)))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
                .build();

        PathChain path5 = new PathBuilder(follower, Constants.pathConstraints)
                .addPath(new BezierCurve(
                        new Pose(89.608, 72.134),
                        new Pose(88.264, 85.128),
                        new Pose(102.153, 83.694)))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
                .build();

        PathChain path6 = new PathBuilder(follower, Constants.pathConstraints)
                .addPath(new BezierLine(
                        new Pose(102.153, 83.694),
                        new Pose(120.433, 83.425)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        PathChain path7 = new PathBuilder(follower, Constants.pathConstraints)
                .addPath(new BezierLine(
                        new Pose(120.433, 83.425),
                        new Pose(88.533, 85.934)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(48))
                .build();

        telemetry.addLine("✅ RB4 paths ready — waiting for start");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // --- Run paths (default speed = 1.0, editable later) ---
        follow(path1, 1.0, "Path 1");
        follow(path2, 1.0, "Path 2");
        follow(path3, 0.33, "Path 3");
        follow(path4, 1.0, "Path 4");
        follow(path5, 1.0, "Path 5");
        follow(path6, 0.25, "Path 6");
        follow(path7, 1.0, "Path 7");

        telemetry.addLine("✅ RB4 Auto Complete");
        telemetry.update();
    }

    // --- Helper method to follow each path ---
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
