package org.firstinspires.ftc.teamcode.auto.paths;

import com.pedropathing.follower.Follower;
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
                        new BezierLine(new Pose(56.000, 8.000), new Pose(35.696, 57.721))
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
                .build();
    }
}