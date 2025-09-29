package org.firstinspires.ftc.teamcode.config.core.paths;

import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;

import java.util.ArrayList;
import java.util.List;

public class FiveSpec {
    public static Pose start = new Pose(9, 65, Math.toRadians(0));
    public static Pose spec1 = new Pose(41, 67, Math.toRadians(0));
    public static Pose grab1 = new Pose(24, 24, Math.toRadians(0));
    public static Pose grab2 = new Pose(23, 14, Math.toRadians(0));
//    public static Pose grab3 = new Pose(23, 13, Math.toRadians(-30));
    // 14.5 -51.4 5.87
    public static Pose grab3 = new Pose(26, 13, Math.toRadians(-28));
    public static Pose pickup1 = new Pose(9, 27, Math.toRadians(0));
//    public static Pose in1 = new Pose(9, 30, Math.toRadians(0));
    public static Pose spec2 = new Pose(41, 69, Math.toRadians(0));
    public static Pose pickup2 = new Pose(9, 34, Math.toRadians(0));
//    public static Pose in2 = new Pose(9, 35, Math.toRadians(0));

    public static List<PathChain> paths = new ArrayList<>();

    public static PathChain spec1() {
        return new PathBuilder()
            .addPath(
                new BezierLine(
                    new Point(start),
                    new Point(spec1)
                )
            )
            .setConstantHeadingInterpolation(start.getHeading())
            .build();
    }

    public static PathChain grab1() {
        return new PathBuilder()
            .addPath(
                new BezierCurve(
                    new Point(spec1),
                    new Point(26,61, Point.CARTESIAN),
                    new Point(grab1)
                )
            )
            .setConstantHeadingInterpolation(grab1.getHeading())
            .build();
    }
    public static PathChain grab2() {
        return new PathBuilder()
            .addPath(
                new BezierLine(
                    new Point(grab1),
                    new Point(grab2)
                )
            )
            .setConstantHeadingInterpolation(grab2.getHeading())
            .setZeroPowerAccelerationMultiplier(2.5)
            .build();
    }
    public static PathChain grab3() {
        return new PathBuilder()
            .addPath(
                new BezierLine(
                    new Point(grab2),
                    new Point(grab3)
                )
            )
            .setLinearHeadingInterpolation(grab2.getHeading(), grab3.getHeading())
            .build();
    }
    public static PathChain pickup1() {
        return new PathBuilder()
            .addPath(
                new BezierLine(
                    new Point(grab3),
                    new Point(pickup1)
                )
            )
            .setLinearHeadingInterpolation(grab3.getHeading(), pickup1.getHeading())
            .setZeroPowerAccelerationMultiplier(1.5)
            .build();
    }
    public static PathChain spec2() {
        return new PathBuilder()
            .addPath(
                new BezierCurve(
                    new Point(pickup1),
                    new Point(pickup1.getX()+6,pickup1.getY(),Point.CARTESIAN),
                    new Point(spec2.getX()-8,spec2.getY(),Point.CARTESIAN),
                    new Point(spec2)
                )
            )
            .setLinearHeadingInterpolation(pickup1.getHeading(), spec2.getHeading())
            .setZeroPowerAccelerationMultiplier(5)
            .build();
    }
    public static PathChain pickup2() {
        return new PathBuilder()
            .addPath(
                new BezierLine(
                    new Point(spec2),
                    new Point(pickup2)
                )
            )
            .setLinearHeadingInterpolation(spec2.getHeading(), pickup2.getHeading())
            .setZeroPowerAccelerationMultiplier(2)
            .build();
    }
    public static PathChain pickup3() {
        return new PathBuilder()
            .addPath(
                new BezierLine(
                    new Point(spec2),
                    new Point(pickup1)
                )
            )
            .setLinearHeadingInterpolation(spec2.getHeading(), pickup1.getHeading())
            .setZeroPowerAccelerationMultiplier(2)
            .build();
    }
    public static PathChain spec3() {
        return new PathBuilder()
            .addPath(
                new BezierCurve(
                    new Point(pickup2),
                    new Point(spec2.getX()-8,spec2.getY(),Point.CARTESIAN),
                    new Point(spec2)
                )
            )
            .setLinearHeadingInterpolation(pickup2.getHeading(), spec2.getHeading())
            .setZeroPowerAccelerationMultiplier(5)
            .build();
    }

    public static PathChain spec2slow() {
        return new PathBuilder()
            .addPath(
                new BezierLine(
                    new Point(pickup1),
                    new Point(spec2)
                )
            )
            .setLinearHeadingInterpolation(pickup1.getHeading(), spec2.getHeading())
            .setZeroPowerAccelerationMultiplier(6)
            .build();
    }

}
