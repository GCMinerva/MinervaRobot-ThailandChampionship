package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.archived.subsystems.ActuatorSubsystem;

/**
 * Helper class for TeleOp ball detection.
 */
public class TeleOpBallDetector {

    private final ActuatorSubsystem actuator;

    public TeleOpBallDetector(ActuatorSubsystem actuator) {
        this.actuator = actuator;
    }

    /**
     * Initializes the ball detection process.
     */
    public void start() {
        actuator.initBallDetection();
    }

    /**
     * Updates the ball detection logic. Call this in a loop.
     */
    public void update() {
        actuator.updateBallDetection();
    }

    /**
     * Stops the intake motor.
     */
    public void stop() {
        actuator.stopIntake();
    }
}
