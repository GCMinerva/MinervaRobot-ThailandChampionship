package org.firstinspires.ftc.teamcode.config.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.config.core.Robot;
import org.firstinspires.ftc.teamcode.config.core.util.Color;

public class ProcessFrame extends CommandBase {
    private final Robot robot;
    private final Color color;
    private final int need;
    private Timer timer = new Timer();
    private boolean frameReceived = false;
    private boolean isFinished = false;

    public ProcessFrame(Robot robot, Color color, int need) {
        this.robot = robot;
        this.color = color;
        this.need = need;
    }

    @Override
    public void initialize() {
        timer.resetTimer();
        robot.getV().setStartPose(robot.getF().getPose());
        robot.getV().setColor(color);
        robot.getV().setNeed(need);
        isFinished = false;
        robot.getV().clear();
        robot.getV().turnOn();
    }

    @Override
    public void execute() {
//        if (!robot.getV().getSamples().isEmpty() || timer.getElapsedTimeSeconds() >= 1) {
        if (robot.getV().cnt >= 1) {
            isFinished = true;
            robot.getV().turnOff();
        }
    }

    @Override
    public boolean isFinished() {
//        return frameReceived && !vision.getSamples().isEmpty();
        return isFinished;
    }

    @Override
    public void end(boolean interrupted) {
        robot.getV().turnOff();
    }
}