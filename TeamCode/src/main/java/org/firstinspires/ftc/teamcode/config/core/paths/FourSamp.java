package org.firstinspires.ftc.teamcode.config.core.paths;

import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;

public class FourSamp {
    public static Pose start = new Pose(9, 113-1, Math.toRadians(0));
    public static Pose score1 = new Pose(16.5-1+1, 132, Math.toRadians(-22)); // -22
    public static Pose transfer2 = new Pose(15.5+5,135,Math.toRadians(-10));
    public static Pose score2 = new Pose(15.5,135,Math.toRadians(-10));
    public static Pose grab3 = new Pose(18-1.5,133,Math.toRadians(18));
//    public static Pose score3 = new Pose(13-1, 134-1,Math.toRadians(-27));
    public static Pose score3 = new Pose(13+2, 134-1+2,Math.toRadians(-15));
    public static Pose grab4 = new Pose(60, 93,Math.toRadians(-90));
    public static Pose grab5 = new Pose(60, 93,Math.toRadians(-90));

    public static PathChain score1() {
        return new PathBuilder()
            .addPath(
                new BezierLine(
                    new Point(start),
                    new Point(score1)
                )
            )
            .setLinearHeadingInterpolation(start.getHeading(), score1.getHeading())
//            .setZeroPowerAccelerationMultiplier(4)
            .build();
    }
    public static PathChain score2() {
        return new PathBuilder()
            .addPath(
                new BezierLine(
                    new Point(transfer2),
                    new Point(score2)
                )
            )
            .setLinearHeadingInterpolation(transfer2.getHeading(), score2.getHeading())
            .setZeroPowerAccelerationMultiplier(4)
            .build();
    }
    public static PathChain grab3() {
        return new PathBuilder()
            .addPath(
                new BezierLine(
                    new Point(score2),
                    new Point(grab3)
                )
            )
            .setLinearHeadingInterpolation(score2.getHeading(), grab3.getHeading())
            .setZeroPowerAccelerationMultiplier(4)
            .build();
    }
    public static PathChain score3() {
        return new PathBuilder()
            .addPath(
                new BezierLine(
                    new Point(grab3),
                    new Point(score3)
                )
            )
            .setLinearHeadingInterpolation(grab3.getHeading(), score3.getHeading())
            .setZeroPowerAccelerationMultiplier(4)
            .build();
    }
    public static PathChain grab4() {
        return new PathBuilder()
            .addPath(
                new BezierCurve(
                    new Point(score3),
                    new Point(60, 125, Point.CARTESIAN),
                    new Point(grab4)
                )
            )
//            .setLinearHeadingInterpolation(score3.getHeading(), grab4.getHeading())
            .setTangentHeadingInterpolation()
            .build();
    }

    public static PathChain score4() {
        return new PathBuilder()
            .addPath(
                new BezierCurve(
                    new Point(grab5),
                    new Point(grab5.getX(), 115, Point.CARTESIAN),
                    new Point(score3)
                )
            )
//            .setLinearHeadingInterpolation(grab4.getHeading(), score3.getHeading())
            .setTangentHeadingInterpolation()
            .setReversed(true)
            .build();
    }

    public static PathChain transfer1() {
        return new PathBuilder()
            .addPath(
                new BezierLine(
                    new Point(score1),
                    new Point(transfer2)
                )
            )
            .setLinearHeadingInterpolation(score1.getHeading(), transfer2.getHeading())
//            .setTangentHeadingInterpolation()
            .build();
    }

    public static PathChain transfer2() {
        return new PathBuilder()
            .addPath(
                new BezierLine(
                    new Point(score2),
                    new Point(transfer2)
                )
            )
            .setLinearHeadingInterpolation(score2.getHeading(), transfer2.getHeading())
//            .setTangentHeadingInterpolation()
            .build();
    }
}
