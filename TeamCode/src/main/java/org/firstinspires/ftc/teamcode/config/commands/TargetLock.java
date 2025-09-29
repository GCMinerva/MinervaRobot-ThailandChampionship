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
import org.firstinspires.ftc.teamcode.config.core.subsystems.Intake;
import org.firstinspires.ftc.teamcode.config.core.vision.Sample;

public class TargetLock extends CommandBase {
    private final Robot robot;
    private Timer timer = new Timer();
    private Pose pose;
    private Sample sample;

    private int state = 0;
    private boolean groundZ;
    private double deltaX;
    private double xMin, xMax;
    private boolean refind, dodge;
    private double lateralMove = 0;

    public TargetLock(Robot robot, boolean groundZ, double deltaX, double xMin, double xMax, boolean refind, boolean dodge) {
        this.robot = robot;
        this.groundZ = groundZ;
        this.deltaX = deltaX;
        this.xMin = xMin;
        this.xMax = xMax;
        this.refind = refind;
        this.dodge = dodge;
        this.pose = null;
        this.lateralMove = 0;
    }

    @Override
    public void initialize() {
        robot.getF().setMaxPower(0.8);
        sample = robot.getV().getBestSample(xMin, xMax, deltaX, true);
        if (sample == null) {
            setState(-1);
            return;
        }
        if (refind) {
            robot.getV().setStartPose(robot.getF().getPose());
        }
        if (dodge) {
            robot.getI().setRotateState(Intake.RotateState.DODGE);
        }
        pose = toSamplePose(robot.getV().getStartPose(),sample,deltaX);
        if (pose != null) {
            lateralMove = Math.hypot(Math.abs(pose.getX()-robot.getF().getPose().getX()),
                Math.abs(pose.getY()-robot.getF().getPose().getY()));
        }
        PathChain path = new PathBuilder()
            .addPath(
                new BezierLine(
                    new Point(robot.getF().getPose()),
                    new Point(pose)
                )
            )
            .setConstantHeadingInterpolation(robot.getF().getPose().getHeading())
            .build();
//        if (groundZ) {
//            robot.getI().groundoZ(true);
//        } else {
//            robot.getI().hovero(true);
//        }
        robot.getF().followPath(path, true);
        setState(1);
    }

    @Override
    public void execute() {
//        robot.getF().update();
        switch (state) {
            case 1:
                if (pose != null) {
                    setState(2);
                }
                break;
            case 2:
                if ((robot.getF().getCurrentTValue() > 0.9 || timer.getElapsedTimeSeconds() > 0.075*lateralMove*0.8)
                    && (dodge ? timer.getElapsedTimeSeconds() > 0.1 : true)) {
                    robot.getE().toInches(sample.getY(), deltaX);
                    robot.getI().hovero(true,robot.getE().getPos());
                    setState(3);
                }
                break;
            case 3:
                if (timer.getElapsedTimeSeconds() > 0) {
                    setState(4);
                }
                break;
            case 4:
                if (timer.getElapsedTimeSeconds() > 0.3) {
                    robot.getI().rotateDegrees(-sample.getAngle()+90.0);
                    setState(5);
                }
                break;
            case 5:
                if ((robot.getF().getCurrentTValue() > 0.99 || timer.getElapsedTimeSeconds() > 0.08*lateralMove)
                    && timer.getElapsedTimeSeconds() > 0.7*(sample.getY()/20.0)-0.3+0.2 && timer.getElapsedTimeSeconds() >= 0) {
//                    robot.getI().rotateDegrees(-sample.getAngle()+90.0);
                    setState(6);
                }
                break;
            case 6:
                if (timer.getElapsedTimeSeconds() >= 0) {
                    if (groundZ) {
                        robot.getI().groundoZ(false,robot.getE().getPos());
                        setState(7);
                    } else {
                        setState(-1);
                    }
                }
                break;
            case 7:
                if (timer.getElapsedTimeSeconds() > 0.2) {
                    setState(-1);
                }
        }
    }

    @Override
    public boolean isFinished() {
//        return frameReceived && !vision.getSamples().isEmpty();
//        return robot.getF().getCurrentTValue() > 0.99;
        return state == -1;
    }

    @Override
    public void end(boolean interrupted) {
        this.robot.getF().setMaxPower(1);
    }

    public void setState(int x) {
        state = x;
        timer.resetTimer();
    }
}