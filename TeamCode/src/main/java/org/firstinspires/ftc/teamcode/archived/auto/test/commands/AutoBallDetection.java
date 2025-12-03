package org.firstinspires.ftc.teamcode.archived.auto.test.commands;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.Subsystem;

import org.firstinspires.ftc.teamcode.archived.subsystems.ActuatorSubsystem;

import java.util.Collections;
import java.util.Set;

/**
 * Command that automatically detects and sorts balls using the distance sensor.
 * Runs until 3 balls are collected, 3 seconds elapse, or the command is
 * interrupted.
 */
public class AutoBallDetection implements Command {

    private final ActuatorSubsystem actuator;
    private boolean initialized = false;
    private long startTime = 0;
    private static final long TIMEOUT_MS = 3000; // 3 seconds timeout

    public AutoBallDetection(ActuatorSubsystem actuator) {
        this.actuator = actuator;
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return Collections.emptySet();
    }

    @Override
    public void initialize() {
        actuator.initBallDetection();
        initialized = true;
        startTime = System.currentTimeMillis();
    }

    @Override
    public void execute() {
        // Update ball detection - returns false when 3+ balls collected
        actuator.updateBallDetection();
    }

    @Override
    public boolean isFinished() {
        if (!initialized) {
            return false;
        }

        // Check if timeout has elapsed
        boolean timeoutReached = (System.currentTimeMillis() - startTime) >= TIMEOUT_MS;

        // Command finishes when 3 or more balls are collected OR timeout is reached
        return actuator.isBallsFull() || timeoutReached;
    }

    @Override
    public void end(boolean interrupted) {
        // Ensure intake is stopped when command ends
//        if (!interrupted) {
//            actuator.stopIntake();
//        }
    }
}
