package org.firstinspires.ftc.teamcode.config.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.config.core.Robot;

public class ToScore extends CommandBase {
    private final Robot robot;

    private int state = 0;
    private Timer timer = new Timer();

    public ToScore(Robot robot) {
        this.robot = robot;
    }

    @Override
    public void initialize() {
        setState(1);
        robot.setRobotState(Robot.RobotState.TRANSFER);
    }

    @Override
    public void execute() {
        robot.getT().addData("Score - state", state);
        switch (state) {
            case 1:
                robot.getO().close();
                setState(2);
                break;
            case 2:
                if (timer.getElapsedTimeSeconds() > 0.15) {
                    robot.getL().toHighBasket();
                    robot.getI().open();
                    setState(3);
                }
                break;
            case 3:
                if (timer.getElapsedTimeSeconds() > 0.2) {
                    robot.getO().score();
                    setState(4);
                }
                break;
            case 4:
                if (robot.getL().roughlyAtTarget() || timer.getElapsedTimeSeconds() > 2) {
                    setState(-1);
                }
                break;
        }
    }

    @Override
    public boolean isFinished() {
        return state == -1;
    }

    public void setState(int x) {
        state = x;
        timer.resetTimer();
    }
}