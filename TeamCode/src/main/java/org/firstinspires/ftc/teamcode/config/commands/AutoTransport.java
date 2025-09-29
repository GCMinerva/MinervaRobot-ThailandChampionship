package org.firstinspires.ftc.teamcode.config.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.config.core.Robot;

public class AutoTransport extends CommandBase {
    private final Robot robot;

    private int state = 0;
    private Timer timer = new Timer();
    private int grab = 0;
    private double deg = 0;

    public AutoTransport(Robot robot, int grab, double deg) {
        this.robot = robot;
        this.grab = grab;
        this.deg = deg;
    }

    @Override
    public void initialize() {
        setState(1);
        robot.setRobotState(Robot.RobotState.TRANSFER);
    }

    @Override
    public void execute() {
        robot.getT().addData("t state", state);
        switch (state) {
            case 1:
                //     transferSampleDetected = (intake.getColor() == IntakeColor.BLUE || intake.getColor() == IntakeColor.RED || intake.getColor() == IntakeColor.YELLOW);
                //robot.getL().toZero();
                robot.getI().transfer();
                robot.getO().transfer();
                robot.getL().toTransfer();
                robot.getE().toZero();
                setState(2);
                break;
            case 2:
                if (timer.getElapsedTimeSeconds() > 1-0.2) {
                    robot.getO().close();
                    setState(3);
                }
                break;
            case 3:
                if (timer.getElapsedTimeSeconds() > 0.15) {
                    robot.getI().open();
                    robot.getL().toHighBasket();
                    if (grab > 0) {
                        robot.getE().toAutoFull();
                        robot.getI().hovero(true,robot.getE().getPos());
                        robot.getI().rotateDegrees(deg);
                    }
                    setState(4);
                }
                break;
            case 4:
                if (timer.getElapsedTimeSeconds() > 0.1) {
                    robot.getO().score();
                    setState(5);
                }
                break;
            case 5:
                if (timer.getElapsedTimeSeconds() > 0.6) {
                    if (grab == 1)robot.getI().groundoZ(false,robot.getE().getPos());
                    setState(6);
                }
                break;
            case 6:
                if (timer.getElapsedTimeSeconds() > 0.2) {
                    if (grab == 1) {
                        robot.getI().close();
                    }
                    setState(7);
                }
                break;
            case 7:
                if (robot.getL().roughlyAtTarget() || timer.getElapsedTimeSeconds() > 1) {
                    setState(8);
                }
                break;
            case 8:
                if (timer.getElapsedTimeSeconds() > 0 && robot.getF().getVelocityMagnitude() < 0.5) {
                    robot.getO().openfull();
                    if (grab == 0) {
                        setState(-1);
                    }
                    setState(9);
                }
                break;
            case 9:
                if (timer.getElapsedTimeSeconds() > 0.1) {
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