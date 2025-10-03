package org.firstinspires.ftc.teamcode.config;

import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Main Robot Configuration Class
 * Centralizes all robot subsystems and hardware configurations
 */
public class RobotConfig {

    // Subsystems
    public ShooterConfig shooter;

    private HardwareMap hardwareMap;

    /**
     * Initialize robot configuration
     */
    public RobotConfig(HardwareMap hwMap) {
        this.hardwareMap = hwMap;
        initializeHardware();
    }

    /**
     * Initialize all hardware subsystems
     */
    private void initializeHardware() {
        // Initialize shooter subsystem
        shooter = new ShooterConfig(hardwareMap);
    }

    /**
     * Stop all motors on the robot
     */
    public void stopAllMotors() {
        shooter.stopAll();
    }
}
