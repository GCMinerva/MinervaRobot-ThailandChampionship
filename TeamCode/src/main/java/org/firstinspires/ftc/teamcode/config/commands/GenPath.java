package org.firstinspires.ftc.teamcode.config.commands;

import static org.firstinspires.ftc.teamcode.config.core.vision.Vision.toSamplePose;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.config.core.Robot;
import org.firstinspires.ftc.teamcode.config.core.paths.CVPaths;
import org.firstinspires.ftc.teamcode.config.core.vision.Sample;
import org.firstinspires.ftc.teamcode.config.core.vision.SamplePath;

public class GenPath extends CommandBase {
    private final Robot robot;
    private Timer timer = new Timer();
    private Pose pose;
    private Sample sample;

    private int state = 0;
    private double deltaX;
    private double xMin, xMax;
    private boolean usecurrent;

    public GenPath(Robot robot, double deltaX, double xMin, double xMax, boolean usecurrent) {
        this.robot = robot;
        this.deltaX = deltaX;
        this.xMin = xMin;
        this.xMax = xMax;
        this.pose = null;
        this.usecurrent = usecurrent;
    }

    @Override
    public void initialize() {
        setState(1);
//        robot.getF().setMaxPower(0.8);
        sample = robot.getV().getBestSample(xMin, xMax, deltaX, true);
        if (sample == null) {
            setState(-1);
            return;
        }
        pose = toSamplePose(robot.getV().getStartPose(),sample,deltaX);
        double lateralMove = Math.hypot(Math.abs(pose.getX()-robot.getV().getStartPose().getX()),
            Math.abs(pose.getY()-robot.getV().getStartPose().getY()));
        PathChain path;
        if (!usecurrent) {
            path = new PathBuilder()
                .addPath(
                    new BezierLine(
                        new Point(robot.getV().getStartPose()),
                        new Point(pose)
                    )
                )
                .setConstantHeadingInterpolation(robot.getV().getStartPose().getHeading())
                .build();
        } else {
            path = new PathBuilder()
                .addPath(
                    new BezierLine(
                        new Point(robot.getF().getPose()),
                        new Point(pose)
                    )
                )
                .setLinearHeadingInterpolation(robot.getF().getPose().getHeading(),pose.getHeading())
                .build();
        }
        CVPaths.paths.add(new SamplePath(path,sample,deltaX,lateralMove));
//        if (groundZ) {
//            robot.getI().groundoZ(true);
//        } else {
//            robot.getI().hovero(true);
//        }
//        robot.getF().followPath(path, true);
        setState(2);
    }

    @Override
    public void execute() {
//        robot.getF().update();
        switch (state) {
            case 2:
                setState(-1);
                break;
        }
    }

    @Override
    public boolean isFinished() {
//        return frameReceived && !vision.getSamples().isEmpty();
//        return robot.getF().getCurrentTValue() > 0.99;
        return state == -1;
    }

    @Override
    public void end(boolean interrupted) { }

    public void setState(int x) {
        state = x;
        timer.resetTimer();
    }
}