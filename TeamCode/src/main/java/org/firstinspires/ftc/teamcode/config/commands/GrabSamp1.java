package org.firstinspires.ftc.teamcode.config.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.config.core.Robot;

public class GrabSamp1 extends CommandBase {
    private final Robot robot;

    private int state = 0;
    private Timer timer = new Timer();
    private Timer oTimer = new Timer();

    public GrabSamp1(Robot robot) {
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
//                robot.getI().groundo();
                robot.getI().hovero(true,robot.getE().getPos());
                robot.getI().rotateDegrees(22);
                robot.getI().rotateForward(-0.03);
                setState(2);
                break;
            case 2:
                if ((robot.getF().getCurrentTValue() > 0.99 && robot.getF().getVelocityMagnitude() < 0.25 && robot.getF().headingError < Math.toRadians(2))
                || timer.getElapsedTimeSeconds() > 2.5) {
                    robot.getI().groundoZ(false,robot.getE().getPos());
                    robot.getI().rotateForward(-0.03);
                    setState(3);
                }
                break;
            case 3:
                if (timer.getElapsedTimeSeconds() > 0.15) {
                    robot.getI().close();
                    setState(4);
                }
                break;
            case 4:
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
