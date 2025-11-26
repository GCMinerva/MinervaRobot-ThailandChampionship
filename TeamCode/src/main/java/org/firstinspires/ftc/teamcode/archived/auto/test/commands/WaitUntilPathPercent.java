package org.firstinspires.ftc.teamcode.archived.auto.test.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.follower.Follower;

/**
 * Command that waits until the follower reaches a specific percentage (T-value) along the current path
 * Similar to parametric callbacks in the old system
 */
public class WaitUntilPathPercent extends CommandBase {
    private final Follower follower;
    private final double targetPercent;

    /**
     * @param follower The Pedro Pathing follower
     * @param targetPercent The percentage to wait for (0.0 to 1.0, e.g., 0.5 = 50%)
     */
    public WaitUntilPathPercent(Follower follower, double targetPercent) {
        this.follower = follower;
        this.targetPercent = Math.max(0.0, Math.min(1.0, targetPercent));
    }

    @Override
    public boolean isFinished() {
        return follower.getCurrentTValue() >= targetPercent;
    }
}
