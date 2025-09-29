package org.firstinspires.ftc.teamcode.config.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.config.core.Robot;

public class Bucket1 extends CommandBase {
    private final Robot robot;

    private int state = 0;
    private Timer timer = new Timer();
    private Timer oTimer = new Timer();

    public Bucket1(Robot robot) {
        this.robot = robot;
    }

    @Override
    public void initialize() {
        setState(1);
    }

    @Override
    public void execute() {
        robot.getT().addData("Bucket - state: ",state);
        switch (state) {
            case 1:
                robot.getE().toAutoFull();
                robot.getL().toHighBasket();
                robot.getO().close();
                setState(2);
                break;
            case 2:
                if (timer.getElapsedTimeSeconds() > 0.8+0.1) {
                    robot.getO().score();
                    robot.getO().close();
                    setState(3);
                }
                break;
            case 3:
                if (robot.getL().roughlyAtTarget() || timer.getElapsedTimeSeconds() > 2) {
                    setState(4);
                }
                break;
            case 4:
                if (timer.getElapsedTimeSeconds() > 0.3 && robot.getF().getVelocityMagnitude() < 0.25) {
                    robot.getO().openfull();
                    setState(5);
                }
                break;
            case 5:
                if (timer.getElapsedTimeSeconds() > 0.15) {
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
