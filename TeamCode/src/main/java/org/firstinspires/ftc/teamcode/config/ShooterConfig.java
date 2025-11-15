package org.firstinspires.ftc.teamcode.config;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.RobotHardwareConfig;

/**
 * Shooter Configuration Class
 * Manages all shooter-related hardware and mechanisms
 * Uses centralized RobotHardwareConfig for all hardware constants
 */
public class ShooterConfig {

    // Motors
    public DcMotor shooterMotor;        // Main shooter motor
    public DcMotor negaShooterMotor;    // Stabilizer motor (counter-rotation)
    public DcMotor outtakeMotor;        // Feed motor (ball intake)

    // Servo
    public Servo shooterServo;          // Shooting mechanism servo

    // Target position (shooting point center) - from config
    private double targetX = RobotHardwareConfig.ShooterParams.TARGET_X;
    private double targetY = RobotHardwareConfig.ShooterParams.TARGET_Y;

    // Distance and power calibration - from config
    private double minDistance = RobotHardwareConfig.ShooterParams.MIN_DISTANCE;
    private double minPower = RobotHardwareConfig.ShooterParams.MIN_POWER;
    private double maxDistance = RobotHardwareConfig.ShooterParams.MAX_DISTANCE;
    private double maxPower = RobotHardwareConfig.ShooterParams.MAX_POWER;

    // Servo positions - from config
    public static final double SERVO_SHOOT = RobotHardwareConfig.ShooterServos.SHOOT_POSITION;
    public static final double SERVO_RESET = RobotHardwareConfig.ShooterServos.RESET_POSITION;

    // Motor power settings - from config
    public static final double NEGA_SHOOTER_POWER = RobotHardwareConfig.ShooterParams.STABILIZER_POWER;
    public static final double OUTTAKE_FEED_POWER = RobotHardwareConfig.ShooterParams.FEED_POWER;
    public static final double OUTTAKE_REVERSE_POWER = RobotHardwareConfig.ShooterParams.REVERSE_FEED_POWER;

    private boolean isReady = true;

    /**
     * Initialize shooter hardware
     */
    public ShooterConfig(HardwareMap hardwareMap) {
        // Initialize motors using names from config
        shooterMotor = hardwareMap.get(DcMotor.class, RobotHardwareConfig.ShooterMotors.MAIN_SHOOTER);
        negaShooterMotor = hardwareMap.get(DcMotor.class, RobotHardwareConfig.ShooterMotors.STABILIZER);
        outtakeMotor = hardwareMap.get(DcMotor.class, RobotHardwareConfig.ShooterMotors.OUTTAKE);

        // Initialize servo using name from config
        shooterServo = hardwareMap.get(Servo.class, RobotHardwareConfig.ShooterServos.SHOOTER_SERVO);

        // Set motor directions from config
        shooterMotor.setDirection(RobotHardwareConfig.ShooterDirections.MAIN_SHOOTER);
        negaShooterMotor.setDirection(RobotHardwareConfig.ShooterDirections.STABILIZER);
        outtakeMotor.setDirection(RobotHardwareConfig.ShooterDirections.OUTTAKE);

        // Set initial servo position
        shooterServo.setPosition(SERVO_RESET);
    }

    /**
     * Set the target shooting position (center point)
     */
    public void setTargetPosition(double x, double y) {
        this.targetX = x;
        this.targetY = y;
    }

    /**
     * Set calibration values for distance and power
     */
    public void setCalibration(double minDist, double minPwr, double maxDist, double maxPwr) {
        this.minDistance = minDist;
        this.minPower = minPwr;
        this.maxDistance = maxDist;
        this.maxPower = maxPwr;
    }

    /**
     * Calculate distance from current position to target
     */
    public double calculateDistance(Pose currentPose) {
        double dx = currentPose.getX() - targetX;
        double dy = currentPose.getY() - targetY;
        return Math.sqrt(dx * dx + dy * dy);
    }

    /**
     * Calculate motor power based on distance using linear interpolation
     */
    public double calculatePower(double distance) {
        // Use centralized calculation method
        return RobotHardwareConfig.calculateShooterPower(distance);
    }

    /**
     * Check if shooter is within range
     */
    public boolean isInRange(Pose currentPose) {
        double distance = calculateDistance(currentPose);
        return RobotHardwareConfig.isInShootingRange(distance);
    }

    /**
     * Prepare shooter with calculated power based on current position
     * Spins up both shooter and stabilizer motors
     */
    public void prepareShot(Pose currentPose) {
        double distance = calculateDistance(currentPose);
        double power = calculatePower(distance);

        // Spin up main shooter
        shooterMotor.setPower(power);

        // Spin up stabilizer at full power (always 1.0)
        negaShooterMotor.setPower(NEGA_SHOOTER_POWER);

        isReady = true;
    }

    /**
     * Manually set shooter power (for testing)
     */
    public void setShooterPower(double power) {
        shooterMotor.setPower(power);
        // Stabilizer always runs at full power (1.0)
        negaShooterMotor.setPower(NEGA_SHOOTER_POWER);
        isReady = true;
    }

    /**
     * Feed ball into shooter using outtake motor
     */
    public void feedBall() {
        outtakeMotor.setPower(OUTTAKE_FEED_POWER);
    }

    /**
     * Stop feeding ball
     */
    public void stopFeed() {
        outtakeMotor.setPower(0);
    }

    /**
     * Reverse outtake (eject ball)
     */
    public void reverseFeed() {
        outtakeMotor.setPower(OUTTAKE_REVERSE_POWER);
    }

    /**
     * Reset servo to prepare for next ball
     */
    public void reset() {
        shooterServo.setPosition(SERVO_RESET);
        stopFeed();
        isReady = true;
    }

    /**
     * Set servo position directly (for tuning)
     */
    public void setServoPosition(double position) {
        shooterServo.setPosition(position);
    }

    /**
     * Toggle servo between shoot and reset positions (for tuning)
     */
    public void toggleServo() {
        if (shooterServo.getPosition() < 0.5) {
            shooterServo.setPosition(SERVO_SHOOT);
        } else {
            shooterServo.setPosition(SERVO_RESET);
        }
    }

    /**
     * Get current servo position
     */
    public double getServoPosition() {
        return shooterServo.getPosition();
    }

    /**
     * Stop all shooter motors
     */
    public void stopAll() {
        shooterMotor.setPower(0);
        negaShooterMotor.setPower(0);
        outtakeMotor.setPower(0);
        isReady = true;
    }

    /**
     * Get current distance to target
     */
    public double getDistanceToTarget(Pose currentPose) {
        return calculateDistance(currentPose);
    }

    /**
     * Get calculated power for current position
     */
    public double getCalculatedPower(Pose currentPose) {
        double distance = calculateDistance(currentPose);
        return calculatePower(distance);
    }

    /**
     * Check if servo is ready to shoot
     */
    public boolean isReadyToShoot() {
        return isReady;
    }

    /**
     * Get target X position
     */
    public double getTargetX() {
        return targetX;
    }

    /**
     * Get target Y position
     */
    public double getTargetY() {
        return targetY;
    }

    /**
     * Simple shoot function for compatibility
     * Sets servo to shoot position and feeds ball
     */
    public void shoot() {
        if (isReady) {
            shooterServo.setPosition(SERVO_SHOOT);
            feedBall();
            isReady = false;
        }
    }

    /**
     * Start shooting sequence
     * Call this once to begin, then call updateShootingSequence() in loop
     */
    public void startShootingSequence(double power) {
        shooterMotor.setPower(power);
        negaShooterMotor.setPower(power);
    }

    /**
     * Move servo up (to 90 degrees / position 0.5)
     */
    public void servoUp() {
        shooterServo.setPosition(RobotHardwareConfig.ShooterServos.UP_POSITION);
    }

    /**
     * Move servo down (to 0 degrees / position 0.0)
     */
    public void servoDown() {
        shooterServo.setPosition(RobotHardwareConfig.ShooterServos.DOWN_POSITION);
    }

    /**
     * Activate outtake motor
     */
    public void activateOuttake() {
        outtakeMotor.setPower(1.0);
    }

    /**
     * Deactivate outtake motor
     */
    public void deactivateOuttake() {
        outtakeMotor.setPower(0);
    }

    /**
     * LOADER FUNCTION - ON
     * Turns on outtake motor at speed 1.0
     */
    public void loader_on() {
        outtakeMotor.setPower(1.0);
    }

    /**
     * LOADER FUNCTION - OFF
     * Turns off outtake motor
     */
    public void loader_off() {
        outtakeMotor.setPower(0);
    }
}
