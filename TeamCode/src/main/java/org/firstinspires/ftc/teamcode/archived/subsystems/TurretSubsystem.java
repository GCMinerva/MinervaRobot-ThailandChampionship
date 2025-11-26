package org.firstinspires.ftc.teamcode.archived.subsystems;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.RobotHardwareConfig;

/**
 * Turret Subsystem (Archived)
 * 
 * Controls a GoBilda motor with encoder feedback to precisely align the turret
 * toward Decode FTC Goal locations using closed-loop position control.
 *
 * The turret coordinate system uses the robot's heading as 0°, with a ±90° range
 * covering Quadrants 1 (right/+90°) and 2 (left/-90°) relative to robot heading.
 *
 * Coordinate System:
 * - 0°   = Robot heading (forward direction)
 * - +90° = Right side (Q1 relative to robot)
 * - -90° = Left side (Q2 relative to robot)
 * 
 * Encoder Calibration:
 * - Default: GoBilda 223 RPM motor: ~28.44 ticks per degree (10,240 ticks per revolution)
 * - Adjust TICKS_PER_DEGREE using TurretCalibration.java OpMode
 * 
 * Calibration Steps:
 * 1. Run TurretCalibration OpMode
 * 2. Rotate turret exactly 360° using gamepad
 * 3. Note total encoder ticks
 * 4. Calculate: ticks_per_degree = total_ticks / 360
 * 5. Update TICKS_PER_DEGREE below with your measured value
 * 
 * Usage:
 * 1. Set goal position: turret.setGoalPosition(goalX, goalY)
 * 2. In autonomous loop: turret.update(robotPose)
 * 3. Check alignment: if (turret.isAligned()) { shoot(); }
 *
 * Encoder allows precise positioning with feedback control.
 */
public class TurretSubsystem {

    public DcMotor turretMotor;

    // ===== ENCODER CALIBRATION - MEASURED VALUE =====
    // Calibrated using TurretCalibration.java: 360° = 2231 ticks
    // GoBilda 5203: 2231 / 360 = 6.19 ticks/degree
    public static final double TICKS_PER_DEGREE = 6.19;

    // Goal position in field coordinates (inches)
    private double goalX = 0.0;
    private double goalY = 0.0;

    // Current turret angle relative to robot heading (degrees)
    private double currentAngle = 0.0;
    private double targetAngle = 0.0;

    // Control parameters
    private double maxPower = 1.0;
    private double minPower = 0.2;
    private double alignmentTolerance = 2.0;  // Degrees
    private double minAngle = -90.0;          // Left limit (Q2)
    private double maxAngle = 90.0;           // Right limit (Q1)

    // PID control
    private double kP = 0.01;  // Proportional gain (power per degree error)
    private double kI = 0.0;   // Integral gain (typically 0 for motor control)
    private double kD = 0.001; // Derivative gain (damping)

    private double integralError = 0.0;
    private double lastError = 0.0;

    /**
     * Initialize turret subsystem
     * @param hardwareMap FTC hardware map
     */
    public TurretSubsystem(HardwareMap hardwareMap) {
        turretMotor = hardwareMap.get(DcMotor.class, "turret");
        
        // Reset encoder to zero
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        // Use RUN_TO_POSITION mode for closed-loop encoder control
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        // Set initial position to zero (0 degrees)
        turretMotor.setTargetPosition(0);
        turretMotor.setPower(0.0);
    }

    /**
     * Set the goal position in field coordinates.
     * The turret will calculate the angle needed to point at this position
     * relative to the robot's current heading.
     *
     * @param goalX X position of goal in field coordinates (inches)
     * @param goalY Y position of goal in field coordinates (inches)
     */
    public void setGoalPosition(double goalX, double goalY) {
        this.goalX = goalX;
        this.goalY = goalY;
    }

    /**
     * Set the starting angle of the turret when robot initializes.
     * Use this to position the turret at a specific angle relative to robot's forward direction.
     * 
     * @param startingAngleDegrees Angle in degrees (-90 to +90)
     *                            0° = straight ahead
     *                            +90° = full right
     *                            -90° = full left
     */
    public void setStartingAngle(double startingAngleDegrees) {
        // Clamp to physical limits
        double clampedAngle = Math.max(-90.0, Math.min(90.0, startingAngleDegrees));
        
        // Convert to encoder ticks
        int targetTicks = (int)(clampedAngle * TICKS_PER_DEGREE);
        
        // Set motor to target position
        turretMotor.setTargetPosition(targetTicks);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setPower(0.25);  // Lower power for initial positioning
        
        // Wait for motor to reach position
        while (turretMotor.isBusy()) {
            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
        
        // Stop motor and reset for normal operation
        turretMotor.setPower(0.0);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /**
     * Update turret alignment based on robot's current pose.
     * Call this continuously in your autonomous loop while following paths.
     *
     * Uses closed-loop encoder feedback (RUN_TO_POSITION) with PID control
     * for precise angle positioning.
     *
     * @param robotPose Current robot pose from odometry (contains X, Y, heading)
     * @return true if turret is aligned within tolerance, false otherwise
     */
    public boolean update(Pose robotPose) {
        // Get robot position and heading
        double robotX = robotPose.getX();
        double robotY = robotPose.getY();
        double robotHeading = robotPose.getHeading(); // In radians from Pedro

        // Vector from robot to goal
        double deltaX = goalX - robotX;
        double deltaY = goalY - robotY;

        // Angle to goal in field coordinates (atan2 returns radians)
        double angleToGoalRad = Math.atan2(deltaY, deltaX);

        // Calculate relative angle (robot heading - goal angle)
        // This gives us the turret angle relative to robot's forward direction
        double relativeAngleRad = robotHeading - angleToGoalRad;

        // Normalize to [-π, π] range to avoid wraparound issues
        while (relativeAngleRad > Math.PI) {
            relativeAngleRad -= 2.0 * Math.PI;
        }
        while (relativeAngleRad < -Math.PI) {
            relativeAngleRad += 2.0 * Math.PI;
        }

        // Convert to degrees and store as target
        targetAngle = relativeAngleRad * 180.0 / Math.PI;

        // Clamp to turret's physical limits (±90°)
        targetAngle = Math.max(minAngle, Math.min(maxAngle, targetAngle));

        // === Encoder-based Position Control ===
        // Convert target angle to encoder ticks
        int targetTicks = (int)(targetAngle * TICKS_PER_DEGREE);
        
        // Set motor target position
        turretMotor.setTargetPosition(targetTicks);
        
        // Get current encoder position and convert to degrees
        int currentTicks = turretMotor.getCurrentPosition();
        currentAngle = currentTicks / TICKS_PER_DEGREE;

        // Calculate error
        double error = targetAngle - currentAngle;

        // Normalize error to [-180, 180] for shortest path
        while (error > 180.0) {
            error -= 360.0;
        }
        while (error < -180.0) {
            error += 360.0;
        }

        // === PID Control ===
        // Proportional term
        double pOutput = kP * error;

        // Integral term (accumulate error, bounded)
        integralError += error;
        integralError = Math.max(-100, Math.min(100, integralError)); // Prevent windup
        double iOutput = kI * integralError;

        // Derivative term (rate of change)
        double dOutput = kD * (error - lastError);
        lastError = error;

        // Total output
        double power = pOutput + iOutput + dOutput;

        // Clamp power to valid range
        power = Math.max(-maxPower, Math.min(maxPower, power));

        // Apply minimum power to overcome friction (if moving)
        if (Math.abs(error) > alignmentTolerance && Math.abs(power) < minPower && Math.abs(power) > 0.01) {
            power = (power > 0 ? minPower : -minPower);
        }

        // Set motor power
        turretMotor.setPower(Math.abs(power));

        // Check if aligned within tolerance
        return Math.abs(error) <= alignmentTolerance;
    }

    /**
     * Check if turret is aligned to target goal
     * @return true if within alignment tolerance
     */
    public boolean isAligned() {
        double error = Math.abs(targetAngle - currentAngle);
        // Account for wraparound
        if (error > 180.0) {
            error = 360.0 - error;
        }
        return error <= alignmentTolerance;
    }

    /**
     * Get current turret angle relative to robot heading (degrees)
     * @return angle in degrees: -90° (left/Q2) to +90° (right/Q1)
     */
    public double getCurrentAngle() {
        return currentAngle;
    }

    /**
     * Get target turret angle (degrees)
     */
    public double getTargetAngle() {
        return targetAngle;
    }

    /**
     * Get angle error between target and current (degrees)
     */
    public double getAngleError() {
        double error = targetAngle - currentAngle;
        // Normalize for shortest path
        while (error > 180.0) {
            error -= 360.0;
        }
        while (error < -180.0) {
            error += 360.0;
        }
        return error;
    }

    /**
     * Get the goal position (X coordinate)
     */
    public double getGoalX() {
        return goalX;
    }

    /**
     * Get the goal position (Y coordinate)
     */
    public double getGoalY() {
        return goalY;
    }

    /**
     * Stop the turret motor
     */
    public void stop() {
        turretMotor.setPower(0.0);
        targetAngle = currentAngle;
    }

    /**
     * Configure control sensitivity and limits
     * @param newMaxPower Maximum motor power (0.0-1.0)
     * @param newMinPower Minimum motor power (0.0-1.0)
     * @param newTolerance Alignment tolerance in degrees
     * @param newKP Proportional gain (power per degree error)
     * @param newKI Integral gain (usually 0)
     * @param newKD Derivative gain (damping)
     */
    public void configureControl(double newMaxPower, double newMinPower, 
                                double newTolerance, double newKP, double newKI, double newKD) {
        this.maxPower = newMaxPower;
        this.minPower = newMinPower;
        this.alignmentTolerance = newTolerance;
        this.kP = newKP;
        this.kI = newKI;
        this.kD = newKD;
    }

    /**
     * Adjust PID gains for tuning
     * @param kP Proportional gain
     * @param kI Integral gain
     * @param kD Derivative gain
     */
    public void setPIDGains(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    /**
     * Reset turret to center (0° = robot heading)
     */
    public void resetToCenter() {
        currentAngle = 0.0;
        targetAngle = 0.0;
        turretMotor.setPower(0.0);
    }
}
