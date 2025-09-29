package org.firstinspires.ftc.teamcode.config.commands;

import static org.firstinspires.ftc.teamcode.config.core.RobotConstants.extendFull;
import static org.firstinspires.ftc.teamcode.config.core.RobotConstants.extendZero;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.config.core.Robot;

public class Shrink extends CommandBase {
    private final Robot robot;

    private int state = 0;
    private Timer timer = new Timer();
    private double waitTime = 0;

    public Shrink(Robot robot) {
        this.robot = robot;
    }

    @Override
    public void initialize() {
        setState(1);
        waitTime = 0.7*(robot.getE().getPos()-extendZero)/(extendFull-extendZero);
    }

    @Override
    public void execute() {
        robot.getT().addData("Shrink - state: ",state);
        switch (state) {
            case 1:
                robot.getE().toZero();
                setState(2);
                break;
            case 2:
                if (timer.getElapsedTimeSeconds() >= waitTime) {
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
