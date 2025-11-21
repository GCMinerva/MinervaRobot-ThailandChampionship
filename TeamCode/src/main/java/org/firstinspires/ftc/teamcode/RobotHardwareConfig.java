package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Centralized Hardware Configuration for Minerva Robot
 * This file contains all hardware device names, motor directions, servo positions,
 * and other hardware-related constants used across the entire robot codebase.
 */
public class RobotHardwareConfig {

    // ==========================
    // DRIVETRAIN CONFIGURATION
    // ==========================

    /** Drivetrain Motor Names */
    public static final class DriveMotors {
        public static final String FRONT_LEFT = "fl";
        public static final String BACK_LEFT = "bl";
        public static final String FRONT_RIGHT = "fr";
        public static final String BACK_RIGHT = "br";
    }

    /** Drivetrain Motor Directions */
    public static final class DriveDirections {
        public static final DcMotorSimple.Direction FRONT_LEFT = DcMotorSimple.Direction.REVERSE;
        public static final DcMotorSimple.Direction BACK_LEFT = DcMotorSimple.Direction.REVERSE;
        public static final DcMotorSimple.Direction FRONT_RIGHT = DcMotorSimple.Direction.FORWARD;
        public static final DcMotorSimple.Direction BACK_RIGHT = DcMotorSimple.Direction.FORWARD;
    }

    /** Drivetrain Parameters */
    public static final class DriveParams {
        public static final double MAX_POWER = 1.0;
        public static final double X_VELOCITY = 69.87935384114584; // in/s forward velocity
        public static final double Y_VELOCITY = 59.3845620143; // in/s strafe velocity
    }

    // ==========================
    // SHOOTER CONFIGURATION
    // ==========================

    /** Shooter Motor Names */
    public static final class ShooterMotors {
        public static final String MAIN_SHOOTER = "shooter";
        public static final String STABILIZER = "negashooter";
        public static final String OUTTAKE = "outtake";
    }

    /** Shooter Motor Directions */
    public static final class ShooterDirections {
        public static final DcMotorSimple.Direction MAIN_SHOOTER = DcMotorSimple.Direction.REVERSE;
        public static final DcMotorSimple.Direction STABILIZER = DcMotorSimple.Direction.FORWARD;
        public static final DcMotorSimple.Direction OUTTAKE = DcMotorSimple.Direction.REVERSE;
    }

    /** Shooter Servo Configuration */
    public static final class ShooterServos {
        public static final String SHOOTER_SERVO = "shserver";

        // Servo Positions
        public static final double RESET_POSITION = 0.0;  // 0 degrees
        public static final double SHOOT_POSITION = 0.9;  // ~81 degrees
        public static final double UP_POSITION = 0.5;     // 90 degrees (manual control)
        public static final double DOWN_POSITION = 0.0;   // 0 degrees
    }

    /** Shooter Power and Distance Parameters */
    public static final class ShooterParams {
        // Power settings
        public static final double MIN_POWER = 0.3;
        public static final double MAX_POWER = 0.9;
        public static final double STABILIZER_POWER = 1.0;
        public static final double FEED_POWER = 1.0;
        public static final double REVERSE_FEED_POWER = -1.0;
        public static final double POWER_INCREMENT = 0.05;

        // Distance parameters (inches)
        public static final double MIN_DISTANCE = 20.0;
        public static final double MAX_DISTANCE = 60.0;

        // Target position (Blue Alliance top left)
        public static final double TARGET_X = 11.0;
        public static final double TARGET_Y = 135.0;

        // Timing delays (milliseconds)
        public static final long SPINUP_DELAY = 500;
        public static final long SHOOT_DELAY = 300;
    }

    // ==========================
    // ODOMETRY CONFIGURATION
    // ==========================

    /** Pinpoint Odometry Configuration */
    public static final class Odometry {
        public static final String PINPOINT_NAME = "pinpoint";

        // Pod offsets (inches)
        public static final double FORWARD_POD_Y_OFFSET = 0.0;
        public static final double STRAFE_POD_X_OFFSET = 0.0;
    }

    // ==========================
    // VISION CONFIGURATION
    // ==========================

    /** Camera Configuration */
    public static final class Camera {
        public static final String WEBCAM_NAME = "Webcam 1";

        // Resolution
        public static final int RESOLUTION_WIDTH = 640;
        public static final int RESOLUTION_HEIGHT = 480;

        // Calibration parameters
        public static final double FOCAL_LENGTH_X = 1449.18;  // pixels
        public static final double FOCAL_LENGTH_Y = 1449.18;  // pixels
        public static final double OPTICAL_CENTER_X = 954.863; // pixels
        public static final double OPTICAL_CENTER_Y = 544.831; // pixels

        // Distortion coefficients
        public static final double RADIAL_K1 = 0.0421727;
        public static final double RADIAL_K2 = -0.205409;
        public static final double RADIAL_K3 = 0.184011;
        public static final double TANGENTIAL_P1 = -0.00186866;
        public static final double TANGENTIAL_P2 = -0.00161809;
        public static final double SKEW = 0.0;
    }

    /** AprilTag Detection Configuration */
    public static final class AprilTag {
        public static final int[] VALID_TAG_IDS = {21, 22, 23};
        public static final int DECIMATION = 2;
        public static final int DEFAULT_TAG_ID = -1;
    }

    /** Ball Detection Configuration */
    public static final class BallDetection {
        public static final String MODEL_FILE = "model.tflite";
        public static final int INPUT_SIZE = 1280;
        public static final float CONFIDENCE_THRESHOLD = 0.5f;
        public static final int MAX_DETECTIONS = 10;

        // Class indices
        public static final int PURPLE_BALL_INDEX = 0;
        public static final int GREEN_BALL_INDEX = 1;
    }

    // ==========================
    // PEDRO PATHING CONFIGURATION
    // ==========================

    /** Robot Physical Parameters */
    public static final class RobotPhysics {
        public static final double MASS = 9.24809; // kg
        public static final double FORWARD_ZERO_POWER_ACCEL = -33.725181778;
        public static final double LATERAL_ZERO_POWER_ACCEL = -64.4723949292;
    }

    /** PIDF Coefficients */
    public static final class PIDFCoefficients {
        // Translational PIDF
        public static final double TRANS_P = 0.1;
        public static final double TRANS_I = 0.0;
        public static final double TRANS_D = 0.01;
        public static final double TRANS_F = 0.003;

        // Heading PIDF
        public static final double HEADING_P = 1.0;
        public static final double HEADING_I = 0.0;
        public static final double HEADING_D = 0.0;
        public static final double HEADING_F = 0.005;

        // Drive PIDF (filtered)
        public static final double DRIVE_P = 0.5;
        public static final double DRIVE_I = 0.001;
        public static final double DRIVE_D = 0.001;
        public static final double DRIVE_F = 0.6;
        public static final double DRIVE_FILTER_DECAY = 0.75;

        // Centripetal scaling
        public static final double CENTRIPETAL_SCALING = 0.0005;
    }

    /** Path Following Constraints */
    public static final class PathConstraints {
        public static final double VELOCITY = 0.99;              // in/s
        public static final double ACCELERATION = 100.0;         // in/s²
        public static final double ANGULAR_VELOCITY = 1.0;       // rad/s
        public static final double ANGULAR_ACCELERATION = 1.0;   // rad/s²
    }

    // ==========================
    // AUTONOMOUS CONFIGURATION
    // ==========================

    /** Autonomous Starting Positions */
    public static final class AutoStartPositions {
        public static final double DEFAULT_X = 56.0;
        public static final double DEFAULT_Y = 8.0;
        public static final double DEFAULT_HEADING = 90.0; // degrees
    }

    // ==========================
    // ARCHIVED HARDWARE (Not Currently Active)
    // ==========================

    /** Archived Actuator System Configuration */
    public static final class ArchivedActuator {
        // Motor names
        public static final String OUTTAKE1 = "outtake1";
        public static final String OUTTAKE2 = "outtake2";
        public static final String INTAKE = "intake";

        // Motor directions
        public static final DcMotorSimple.Direction OUTTAKE1_DIR = DcMotorSimple.Direction.FORWARD;
        public static final DcMotorSimple.Direction OUTTAKE2_DIR = DcMotorSimple.Direction.REVERSE;

        // Motor powers
        public static final double FAR_SHOT_POWER = -0.8;
        public static final double CLOSE_SHOT_POWER = -0.67;
        public static final double INTAKE_POWER = -0.4;

        // Servo names
        public static final String BINIT_SERVO = "binit";
        public static final String BRINIT_SERVO = "brinit";
        public static final String SORTER_SERVO = "sorter";

        // Binit servo positions
        public static final double BINIT_CLOSED = 0.69;
        public static final double BINIT_OPEN = 0.135;
        public static final double BRINIT_CLOSED = 0.69;
        public static final double BRINIT_OPEN = 0.135;

        public static final double BINIT_INITIAL = 0.35;

        // Sorter field positions
        public static final double SORTER_P1 = 0.751;
        public static final double SORTER_P2 = 0.0;
        public static final double SORTER_P3 = 0.354;

        // Sorter intake positions
        public static final double SORTER_IN1 = 0.15;
        public static final double SORTER_IN2 = 0.533;
        public static final double SORTER_IN3 = 0.918;
    }

    // ==========================
    // UTILITY METHODS
    // ==========================

    /**
     * Check if an AprilTag ID is valid
     */
    public static boolean isValidTagId(int tagId) {
        for (int validId : AprilTag.VALID_TAG_IDS) {
            if (tagId == validId) {
                return true;
            }
        }
        return false;
    }

    /**
     * Calculate shooter power based on distance
     */
    public static double calculateShooterPower(double distance) {
        if (distance < ShooterParams.MIN_DISTANCE) {
            return ShooterParams.MIN_POWER;
        } else if (distance > ShooterParams.MAX_DISTANCE) {
            return ShooterParams.MAX_POWER;
        } else {
            // Linear interpolation
            double range = ShooterParams.MAX_DISTANCE - ShooterParams.MIN_DISTANCE;
            double powerRange = ShooterParams.MAX_POWER - ShooterParams.MIN_POWER;
            double distanceRatio = (distance - ShooterParams.MIN_DISTANCE) / range;
            return ShooterParams.MIN_POWER + (distanceRatio * powerRange);
        }
    }

    /**
     * Check if robot is within shooting range
     */
    public static boolean isInShootingRange(double distance) {
        return distance >= ShooterParams.MIN_DISTANCE && distance <= ShooterParams.MAX_DISTANCE;
    }
}