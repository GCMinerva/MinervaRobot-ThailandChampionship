package org.firstinspires.ftc.teamcode.config.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.config.core.Robot;

public class GrabSpec3 extends CommandBase {
    private final Robot robot;

    private int state = 0;
    private Timer timer = new Timer();

    public GrabSpec3(Robot robot) {
        this.robot = robot;
    }

    @Override
    public void initialize() {
        setState(1);
    }

    @Override
    public void execute() {
        robot.getT().addData("GrabSpecHurry - state: ",state);
        switch (state) {
            case 1:
                robot.getE().toGrab();
                robot.getI().hovero(true,robot.getE().getPos());
                robot.getI().rotateDegrees(28);
                setState(2);
                break;
            case 2:
                if (timer.getElapsedTimeSeconds() >= 0.4+0.2) {
                    robot.getI().groundoZ(false,robot.getE().getPos());
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
                    robot.getE().toZero();
                    robot.getI().throwback();
                    robot.getO().pickup();
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
