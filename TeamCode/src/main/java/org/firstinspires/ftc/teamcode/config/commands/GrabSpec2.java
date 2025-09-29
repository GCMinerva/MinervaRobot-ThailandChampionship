package org.firstinspires.ftc.teamcode.config.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.config.core.Robot;

public class GrabSpec2 extends CommandBase {
    private final Robot robot;

    private int state = 0;
    private Timer timer = new Timer();

    public GrabSpec2(Robot robot) {
        this.robot = robot;
    }

    @Override
    public void initialize() {
        setState(1);
    }

    @Override
    public void execute() {
        robot.getT().addData("GrabSpec2 - state: ",state);
        switch (state) {
            case 1:
                robot.getE().toGrab();
                robot.getI().groundo(true,robot.getE().getPos());
                setState(2);
                break;
            case 2:
                if (timer.getElapsedTimeSeconds() >= 0.4 && robot.getF().getVelocityMagnitude() < 3) {
                    robot.getI().groundoZ(true,robot.getE().getPos());
                    setState(3);
                }
                break;
            case 3:
                if (timer.getElapsedTimeSeconds() > 0.5-0.4) {
                    robot.getI().close();
                    setState(4);
                }
                break;
            case 4:
                if (timer.getElapsedTimeSeconds() > 0.15) {
//                    robot.getE().toHoldback();
//                    robot.getI().setPivotState(Intake.PivotState.HOLDBACK);
                    robot.getE().toZero();
                    robot.getI().throwback();
                    setState(5);
                }
                break;
            case 5:
                if (timer.getElapsedTimeSeconds() > 0) { // 0.35
//                    robot.getI().setRotateState(Intake.RotateState.HOLDBACK);
                    setState(6);
                }
                break;
            case 6:
                if (timer.getElapsedTimeSeconds() >= 0.35) { // 0.12
                    robot.getI().open();
                    setState(7);
                }
                break;
            case 7:
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
