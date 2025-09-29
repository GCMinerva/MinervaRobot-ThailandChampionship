package org.firstinspires.ftc.teamcode.config.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.config.core.Robot;
import org.firstinspires.ftc.teamcode.config.core.subsystems.Intake;

public class Transport extends CommandBase {
    private final Robot robot;

    private int state = 0;
    private Timer timer = new Timer();

    public Transport(Robot robot) {
        this.robot = robot;
    }

    @Override
    public void initialize() {
        setState(1);
    }

    @Override
    public void execute() {
        robot.getT().addData("t state", state);
        switch (state) {
            case 1:
                //     transferSampleDetected = (intake.getColor() == IntakeColor.BLUE || intake.getColor() == IntakeColor.RED || intake.getColor() == IntakeColor.YELLOW);
                //robot.getL().toZero();
                robot.getI().transfer();
                robot.getI().close();
                robot.getO().transfer();
                robot.getL().toTransfer();
                robot.getE().toZero();
                setState(2);
                break;
            case 2:
                if (robot.getI().pivotState != Intake.PivotState.TRANSFER) {
                    setState(-1);
                }
                if (timer.getElapsedTimeSeconds() > 0.5) {
                    robot.getI().loose();
                    setState(3);
                }
                break;
            case 3:
                if (robot.getI().pivotState != Intake.PivotState.TRANSFER) {
                    setState(-1);
                }
                if (timer.getElapsedTimeSeconds() > 1-0.5) {
                    robot.getO().close();
                    setState(4);
                }
                break;
            case 4:
                if (robot.getI().pivotState != Intake.PivotState.TRANSFER) {
                    setState(-1);
                }
                if (timer.getElapsedTimeSeconds() > 0.2) {
                    robot.getI().open();
                    robot.getL().toHighBasket();
                    robot.getO().score();
                    robot.getO().close();
                    setState(5);
                }
                break;
            case 5:
                if (timer.getElapsedTimeSeconds() > 0.2) {
                    robot.getO().loose();
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