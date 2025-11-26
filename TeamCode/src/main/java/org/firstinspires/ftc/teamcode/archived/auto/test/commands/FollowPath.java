package org.firstinspires.ftc.teamcode.archived.auto.test.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;

public class FollowPath extends CommandBase {
    private final Follower follower;
    private final PathChain path;
    private boolean holdEnd = true;
    private double maxPower = 1;
    private double completionThreshold = 0.99;

    public FollowPath(Follower follower, PathChain pathChain) {
        this.follower = follower;
        this.path = pathChain;
    }

    public FollowPath(Follower follower, PathChain pathChain, double maxPower) {
        this.follower = follower;
        this.path = pathChain;
        this.maxPower = maxPower;
    }

    public FollowPath(Follower follower, PathChain pathChain, boolean holdEnd) {
        this.follower = follower;
        this.path = pathChain;
        this.holdEnd = holdEnd;
    }

    /**
     * Sets whether the robot should maintain its position at the end of the path
     */
    public FollowPath setHoldEnd(boolean holdEnd) {
        this.holdEnd = holdEnd;
        return this;
    }

    /**
     * Sets the follower's maximum power (0-1)
     */
    public FollowPath setMaxPower(double power) {
        this.maxPower = power;
        return this;
    }

    /**
     * Sets the T-value at which the path is considered complete (0-1)
     */
    public FollowPath setCompletionThreshold(double t) {
        this.completionThreshold = t;
        return this;
    }

    @Override
    public void initialize() {
        follower.setMaxPower(this.maxPower);
        follower.followPath(path, holdEnd);
    }

    @Override
    public void execute() {
        follower.update();
    }

    @Override
    public boolean isFinished() {
        return follower.getCurrentTValue() >= completionThreshold || !follower.isBusy();
    }

    @Override
    public void end(boolean interrupted) {
        follower.setMaxPower(1);
    }
}
