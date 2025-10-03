package org.firstinspires.ftc.teamcode.auto.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathConstraints;

public class Path2 {

    public static PathChain buildPath(Follower follower, PathConstraints constraints) {
        PathBuilder builder = new PathBuilder(follower, constraints);

        return builder
                .addPath(
                        new BezierCurve(
                                new Pose(55.728, 11.486),
                                new Pose(8.083, 90.399),
                                new Pose(90.186, 86.357),
                                new Pose(89.123, 110.180)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(45))
                .build();
    }
}