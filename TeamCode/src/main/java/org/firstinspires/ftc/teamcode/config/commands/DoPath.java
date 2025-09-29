package org.firstinspires.ftc.teamcode.config.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.config.core.Robot;
import org.firstinspires.ftc.teamcode.config.core.paths.CVPaths;
import org.firstinspires.ftc.teamcode.config.core.subsystems.Intake;
import org.firstinspires.ftc.teamcode.config.core.vision.SamplePath;

public class DoPath extends CommandBase {
    private final Robot robot;
    private Timer timer = new Timer();
    private SamplePath samplePath;
    private int state = 0;
    private boolean groundZ;
    private boolean dodge, failsafe;

    public DoPath(Robot robot, boolean groundZ, boolean dodge, boolean failsafe) {
        this.robot = robot;
        this.groundZ = groundZ;
        this.dodge = dodge;
        this.failsafe = failsafe;
    }

    @Override
    public void initialize() {
        if (CVPaths.paths.isEmpty()) {
            samplePath = null;
            if (failsafe) {
                robot.getE().toFull();
                robot.getI().hovero(true,robot.getE().getPos());
            }
            setState(-1);
            return;
        }
        if (dodge) {
            robot.getI().setRotateState(Intake.RotateState.DODGE);
        }
        samplePath = CVPaths.paths.remove(0);
//        robot.getF().setMaxPower(0.8);
        robot.getF().followPath(samplePath.path, true);
        setState(1);
    }

    @Override
    public void execute() {
//        robot.getF().update();
        switch (state) {
            case 1:
                if ((robot.getF().getCurrentTValue() > 0.9 || timer.getElapsedTimeSeconds() > 0.08*samplePath.lateralMove*0.8)
                    && (dodge ? timer.getElapsedTimeSeconds() > 0.15 : true)) {
                    robot.getE().toInches(samplePath.sample.getY(), samplePath.deltaX);
                    robot.getI().hovero(true,robot.getE().getPos());
                    setState(2);
                }
                break;
            case 2:
                if (timer.getElapsedTimeSeconds() > 0) {
                    setState(3);
                }
                break;
            case 3:
                if (timer.getElapsedTimeSeconds() > 0.3) {
                    robot.getI().rotateDegrees(-samplePath.sample.getAngle()+90.0);
                    setState(4);
                }
                break;
            case 4:
                if ((robot.getF().getCurrentTValue() > 0.99 || timer.getElapsedTimeSeconds() > 0.08*samplePath.lateralMove)
                    && timer.getElapsedTimeSeconds() > 0.7*(samplePath.sample.getY()/20.0)-0.3 && timer.getElapsedTimeSeconds() >= 0) {
//                    robot.getI().rotateDegrees(-sample.getAngle()+90.0);
                    setState(5);
                }
                break;
            case 5:
                if (timer.getElapsedTimeSeconds() >= 0) {
                    if (groundZ) {
                        robot.getI().groundo(false,robot.getE().getPos());
                        setState(6);
                    } else {
                        setState(-1);
                    }
                }
                break;
            case 6:
                if (timer.getElapsedTimeSeconds() > 0.2) {
                    setState(-1);
                }
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
    public void end(boolean interrupted) {
        this.robot.getF().setMaxPower(1);
    }

    public void setState(int x) {
        state = x;
        timer.resetTimer();
    }
}