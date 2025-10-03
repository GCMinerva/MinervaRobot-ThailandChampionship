package org.firstinspires.ftc.teamcode.auto.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathConstraints;

public class Path3 {

    public static PathChain buildPath(Follower follower, PathConstraints constraints) {
        PathBuilder builder = new PathBuilder(follower, constraints);

        return builder
                .addPath(
                        // Path 1
                        new BezierLine(new Pose(64.000, 80.000), new Pose(58.540, 85.868))
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(134))
                .addPath(
                        // Path 2
                        new BezierCurve(
                                new Pose(58.540, 85.868),
                                new Pose(54.053, 83.012),
                                new Pose(42.834, 83.216)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(134), Math.toRadians(180))
                .addPath(
                        // Path 3
                        new BezierLine(new Pose(42.834, 83.216), new Pose(21.622, 83.420))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .addPath(
                        // Path 4
                        new BezierCurve(
                                new Pose(21.622, 83.420),
                                new Pose(24.885, 71.590),
                                new Pose(15.707, 72.610)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .addPath(
                        // Path 5
                        new BezierLine(new Pose(15.707, 72.610), new Pose(58.540, 85.868))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(134))
                .addPath(
                        // Path 6
                        new BezierCurve(
                                new Pose(58.540, 85.868),
                                new Pose(69.758, 56.089),
                                new Pose(42.223, 59.556)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(134), Math.toRadians(180))
                .addPath(
                        // Path 7
                        new BezierLine(new Pose(42.223, 59.556), new Pose(22.030, 59.760))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .addPath(
                        // Path 8
                        new BezierLine(new Pose(22.030, 59.760), new Pose(58.540, 85.868))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(134))
                .addPath(
                        // Path 9
                        new BezierCurve(
                                new Pose(58.540, 85.868),
                                new Pose(70.166, 34.469),
                                new Pose(42.223, 35.284)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(134), Math.toRadians(180))
                .addPath(
                        // Path 10
                        new BezierLine(new Pose(42.223, 35.284), new Pose(21.826, 35.488))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .addPath(
                        // Path 11
                        new BezierLine(new Pose(21.826, 35.488), new Pose(58.540, 85.868))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(134))
                .build();

    }
}