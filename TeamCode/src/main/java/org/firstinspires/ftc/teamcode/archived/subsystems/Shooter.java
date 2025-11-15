package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Shooter {

    // Hardware
    public DcMotor shooterMotor;  // Public for tuning access
    private Servo shooterServo;

    // Target position (shooting point center)
    // Blue alliance top left corner as example (11, 135)
    private double targetX = 11.0;
    private double targetY = 135.0;

    // Distance and power calibration
    private double minDistance = 20.0;  // Minimum shooting distance (inches)
    private double minPower = 0.3;      // Power at minimum distance
    private double maxDistance = 60.0;  // Maximum shooting distance (inches)
    private double maxPower = 0.9;      // Power at maximum distance

    // Servo positions
    private static final double SERVO_SHOOT = 0.9;   // Position to shoot (put ball in shooter)
    private static final double SERVO_RESET = 0.0;   // Position to reset (ready for next ball)

    private boolean isReady = true;

    public Shooter(HardwareMap hardwareMap) {
        // Initialize hardware
        shooterMotor = hardwareMap.get(DcMotor.class, "shooter");
        shooterServo = hardwareMap.get(Servo.class, "shserver");

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
    private double calculateDistance(Pose currentPose) {
        double dx = currentPose.getX() - targetX;
        double dy = currentPose.getY() - targetY;
        return Math.sqrt(dx * dx + dy * dy);
    }

    /**
     * Calculate motor power based on distance using linear interpolation
     */
    private double calculatePower(double distance) {
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
     */
    public void prepareShot(Pose currentPose) {
        double distance = calculateDistance(currentPose);
        double power = calculatePower(distance);
        shooterMotor.setPower(power);
        isReady = true;
    }

    /**
     * Shoot the ball (activate servo)
     * Note: Must call reset() before shooting next ball
     */
    public void shoot() {
        if (isReady) {
            shooterServo.setPosition(SERVO_SHOOT);
            isReady = false;
        }
    }

    /**
     * Reset servo to prepare for next ball
     * Must be called before shoot() can be used again
     */
    public void reset() {
        shooterServo.setPosition(SERVO_RESET);
        isReady = true;
    }

    /**
     * Stop shooter motor
     */
    public void stop() {
        shooterMotor.setPower(0);
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
}
