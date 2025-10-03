package org.firstinspires.ftc.teamcode.config;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Shooter Configuration Class
 * Manages all shooter-related hardware and mechanisms
 *
 * Hardware Components:
 * - shooter: Main shooting motor (power control)
 * - negashooter: Stabilizer motor (keeps mechanism stable)
 * - outtake: Motor to feed ball into shooter
 * - shserver: Servo to control shooting mechanism
 */
public class ShooterConfig {

    // Motors
    public DcMotor shooterMotor;        // Main shooter motor
    public DcMotor negaShooterMotor;    // Stabilizer motor (counter-rotation)
    public DcMotor outtakeMotor;        // Feed motor (ball intake)

    // Servo
    public Servo shooterServo;          // Shooting mechanism servo

    // Target position (shooting point center)
    private double targetX = 11.0;
    private double targetY = 135.0;

    // Distance and power calibration
    private double minDistance = 20.0;  // Minimum shooting distance (inches)
    private double minPower = 0.3;      // Power at minimum distance
    private double maxDistance = 60.0;  // Maximum shooting distance (inches)
    private double maxPower = 0.9;      // Power at maximum distance

    // Servo positions
    public static final double SERVO_SHOOT = 0.9;   // Shooting position
    public static final double SERVO_RESET = 0.0;   // Reset position

    // Motor power settings
    public static final double NEGA_SHOOTER_POWER = 1.0;  // Stabilizer power (always full)
    public static final double OUTTAKE_FEED_POWER = 1.0;  // Power to feed ball in (always full)
    public static final double OUTTAKE_REVERSE_POWER = -1.0; // Power to reverse if needed

    private boolean isReady = true;

    /**
     * Initialize shooter hardware
     */
    public ShooterConfig(HardwareMap hardwareMap) {
        // Initialize motors
        shooterMotor = hardwareMap.get(DcMotor.class, "shooter");
        negaShooterMotor = hardwareMap.get(DcMotor.class, "negashooter");
        outtakeMotor = hardwareMap.get(DcMotor.class, "outtake");

        // Initialize servo
        shooterServo = hardwareMap.get(Servo.class, "shserver");

        // Set motor directions
        shooterMotor.setDirection(DcMotor.Direction.REVERSE);      // Reversed
        negaShooterMotor.setDirection(DcMotor.Direction.FORWARD);
        outtakeMotor.setDirection(DcMotor.Direction.REVERSE);

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
        // Clamp distance to min/max range
        if (distance <= minDistance) {
            return minPower;
        } else if (distance >= maxDistance) {
            return maxPower;
        }

        // Linear interpolation between min and max
        double ratio = (distance - minDistance) / (maxDistance - minDistance);
        return minPower + (ratio * (maxPower - minPower));
    }

    /**
     * Check if shooter is within range
     */
    public boolean isInRange(Pose currentPose) {
        double distance = calculateDistance(currentPose);
        return distance >= minDistance && distance <= maxDistance;
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
     * Shoot the ball (activate servo and feed)
     * Call this after prepareShot() when motors are at speed
     */
    public void shoot() {
        if (isReady) {
            shooterServo.setPosition(SERVO_SHOOT);
            feedBall();
            isReady = false;
        }
    }

    /**
     * Reset servo to prepare for next ball
     * Must be called before shoot() can be used again
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
     * Execute ball shooter sequence
     * Sets motors to specified power and runs outtake with servo cycling
     * @param shooterPower Power for shooter and negashooter motors
     */
    public void startBallShooter(double shooterPower) {
        shooterMotor.setPower(shooterPower);
        negaShooterMotor.setPower(shooterPower);
        shooterServo.setPosition(0.0); // Start position
        outtakeMotor.setPower(0);
        isReady = true;
    }

    /**
     * Execute one cycle of the ball shooter sequence
     * Servo goes to 180 (1.0) for 0.5s, then down, outtake runs for 1.5s, servo up again
     */
    public void servoUp() {
        shooterServo.setPosition(1.0); // 180 degrees
    }

    public void servoDown() {
        shooterServo.setPosition(0.0); // 0 degrees
    }

    public void activateOuttake() {
        outtakeMotor.setPower(1.0);
    }

    public void deactivateOuttake() {
        outtakeMotor.setPower(0);
    }
}
