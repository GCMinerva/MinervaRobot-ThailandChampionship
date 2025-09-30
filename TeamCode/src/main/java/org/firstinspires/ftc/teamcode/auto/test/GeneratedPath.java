
package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;

public class GeneratedPath {

    public static PathBuilder builder = new PathBuilder();

    public static PathChain paths = builder
            .addPath(
                    // Path 1
                    new BezierLine(new Pose(56.000, 8.000), new Pose(35.696, 57.721))
            )
            .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
            .addPath(
                    // Path 2
                    new BezierCurve(
                            new Pose(35.696, 57.721),
                            new Pose(19.582, 139.919),
                            new Pose(130.744, 135.228),
                            new Pose(106.472, 54.865)
                    )
            )
            .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
            .addPath(
                    // Path 3
                    new BezierCurve(
                            new Pose(106.472, 54.865),
                            new Pose(74.653, 15.704),
                            new Pose(31.820, 38.140),
                            new Pose(35.696, 88.724)
                    )
            )
            .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
            .build();
}