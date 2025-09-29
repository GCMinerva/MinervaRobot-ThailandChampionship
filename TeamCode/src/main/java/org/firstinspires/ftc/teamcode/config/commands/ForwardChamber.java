package org.firstinspires.ftc.teamcode.config.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.config.core.Robot;
import org.firstinspires.ftc.teamcode.config.core.subsystems.Intake;

public class ForwardChamber extends CommandBase {
    private final Robot robot;

    private int state = 0;
    private Timer timer = new Timer();

    public ForwardChamber(Robot robot) {
        this.robot = robot;
    }

    @Override
    public void initialize() {
        setState(1);
    }

    @Override
    public void execute() {
        robot.getT().addData("Forward Chamber - state: ",state);
        switch (state) {
            case 1:
                robot.getO().close();
                robot.getI().open();
                setState(2);
                break;
            case 2:
                if (timer.getElapsedTimeSeconds() > 0.15) {
                    robot.getL().toSpecimen();
                    robot.getI().setRotateState(Intake.RotateState.DODGE);
                    setState(3);
                }
                break;
            case 3:
                if (timer.getElapsedTimeSeconds() > 0.4) {
                    robot.getO().specimen();
                    setState(4);
                }
                break;
            case 4:
                if (robot.getF().getCurrentTValue() > 0.95) {
                    setState(5);
                }
                break;
            case 5:
                if (timer.getElapsedTimeSeconds() > 0.1) {
                    robot.getO().open();
                    robot.getL().toPickup();
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
