package org.firstinspires.ftc.teamcode.config.core.paths;

import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;

public class TeleOpPaths {

    public static Pose spec = new Pose(42, 70, Math.toRadians(0));
    public static Pose pickup = new Pose(9, 35, Math.toRadians(0));
    public static Pose score = new Pose(13+2, 134-1+2,Math.toRadians(-15));

    public static PathChain spec() {
        return new PathBuilder()
            .addPath(
                new BezierLine(
                    new Point(pickup),
                    new Point(spec)
                )
            )
            .setLinearHeadingInterpolation(pickup.getHeading(), spec.getHeading())
//            .setTangentHeadingInterpolation()
            .build();
    }
    public static PathChain pickup() {
        return new PathBuilder()
            .addPath(
                new BezierLine(
                    new Point(spec),
                    new Point(pickup)
                )
            )
            .setLinearHeadingInterpolation(spec.getHeading(), pickup.getHeading())
            .build();
    }
}
