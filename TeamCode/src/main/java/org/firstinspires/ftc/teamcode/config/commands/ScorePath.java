package org.firstinspires.ftc.teamcode.config.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.config.core.Robot;

public class ScorePath extends CommandBase {
    private final Robot robot;

    private int state = 0;
    private Timer timer = new Timer();
    private Pose scorePose = new Pose();

    public ScorePath(Robot robot, Pose scorePose) {
        this.robot = robot;
        this.scorePose = scorePose;
    }

    @Override
    public void initialize() {
        PathChain path = new PathBuilder()
            .addPath(
                new BezierCurve(
                    new Point(robot.getF().getPose()),
                    new Point(robot.getF().getPose().getX(), 120, Point.CARTESIAN),
                    new Point(scorePose)
                )
            )
            .setTangentHeadingInterpolation()
            .setReversed(true)
            .build();
        robot.getF().followPath(path, false);
        setState(1);
    }

    @Override
    public void execute() {
        switch (state) {
            case 1:
                if (robot.getF().getCurrentTValue() > 0.95) {
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