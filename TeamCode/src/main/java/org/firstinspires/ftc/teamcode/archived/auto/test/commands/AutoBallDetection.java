package org.firstinspires.ftc.teamcode.archived.auto.test.commands;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.Subsystem;

import org.firstinspires.ftc.teamcode.archived.subsystems.ActuatorSubsystem;

import java.util.Collections;
import java.util.Set;

/**
 * Command that automatically detects and sorts balls using the distance sensor.
 * Runs until 3 balls are collected or the command is interrupted.
 */
public class AutoBallDetection implements Command {

    private final ActuatorSubsystem actuator;
    private boolean initialized = false;

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
        // Command finishes when 3 or more balls are collected
        return actuator.isBallsFull();
    }

    @Override
    public void end(boolean interrupted) {
        // Ensure intake is stopped when command ends
        if (!interrupted) {
            actuator.stopIntake();
        }
    }
}
