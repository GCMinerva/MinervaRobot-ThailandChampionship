package org.firstinspires.ftc.teamcode.config.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.config.core.Robot;

public class Hang extends CommandBase {
    private final Robot robot;

    private int state = 0;
    private Timer timer = new Timer();

    public Hang(Robot robot) {
        this.robot = robot;
    }

    @Override
    public void initialize() {
        setState(1);
    }

    @Override
    public void execute() {
        robot.getT().addData("Hang - state: ",state);
        switch (state) {
            case 1:
                robot.getL().toHang();
                robot.getO().specimen();
                setState(2);
                break;
            case 2:
                if (robot.getL().roughlyAtTarget()) {
                    robot.getP().setPosition(1);
                    setState(3);
                }
                break;
            case 3:
                if (timer.getElapsedTimeSeconds() > 1) {
                    robot.getL().toFinish();
                    setState(4);
                }
                break;
            case 4:
                if (timer.getElapsedTimeSeconds() > 0.3) {
                    robot.getP().setPosition(0.3);
                    setState(-1);
                }
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
