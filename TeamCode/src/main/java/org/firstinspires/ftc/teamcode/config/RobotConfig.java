package org.firstinspires.ftc.teamcode.config;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class RobotConfig {

    // Motors
    public DcMotor intakeMotor;
    public DcMotor outtakeMotor;
    public DcMotor armMotor;
    public DcMotor hangMotor;

    // Servos
    public Servo clawServo;
    public Servo wristServo;
    public Servo intakeServo;

    // Motor positions and powers
    public static final double INTAKE_POWER = 1.0;
    public static final double OUTTAKE_POWER = 1.0;
    public static final double ARM_POWER = 0.8;

    // Servo positions
    public static final double CLAW_OPEN = 0.5;
    public static final double CLAW_CLOSED = 0.0;
    public static final double WRIST_UP = 0.7;
    public static final double WRIST_DOWN = 0.3;
    public static final double INTAKE_SERVO_ACTIVE = 0.6;
    public static final double INTAKE_SERVO_REST = 0.4;

    // Motor encoder positions
    public static final int ARM_LOW_POSITION = 100;
    public static final int ARM_MID_POSITION = 500;
    public static final int ARM_HIGH_POSITION = 1000;

    private HardwareMap hardwareMap;

    public RobotConfig(HardwareMap hwMap) {
        this.hardwareMap = hwMap;
        initializeHardware();
    }

    private void initializeHardware() {
        // Initialize motors
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        outtakeMotor = hardwareMap.get(DcMotor.class, "outtakeMotor");
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        hangMotor = hardwareMap.get(DcMotor.class, "hangMotor");

        // Set motor directions
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        outtakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        armMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        hangMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set zero power behavior
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hangMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set run mode
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Initialize servos
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        wristServo = hardwareMap.get(Servo.class, "wristServo");
        intakeServo = hardwareMap.get(Servo.class, "intakeServo");

        // Set initial servo positions
        clawServo.setPosition(CLAW_OPEN);
        wristServo.setPosition(WRIST_UP);
        intakeServo.setPosition(INTAKE_SERVO_REST);
    }

    // Motor control methods
    public void runIntake(double power) {
        intakeMotor.setPower(power);
    }

    public void runOuttake(double power) {
        outtakeMotor.setPower(power);
    }

    public void setArmPosition(int targetPosition, double power) {
        armMotor.setTargetPosition(targetPosition);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(power);
    }

    public void stopAllMotors() {
        intakeMotor.setPower(0);
        outtakeMotor.setPower(0);
        armMotor.setPower(0);
        hangMotor.setPower(0);
    }

    // Servo control methods
    public void openClaw() {
        clawServo.setPosition(CLAW_OPEN);
    }

    public void closeClaw() {
        clawServo.setPosition(CLAW_CLOSED);
    }

    public void wristUp() {
        wristServo.setPosition(WRIST_UP);
    }

    public void wristDown() {
        wristServo.setPosition(WRIST_DOWN);
    }

    public void activateIntakeServo() {
        intakeServo.setPosition(INTAKE_SERVO_ACTIVE);
    }

    public void restIntakeServo() {
        intakeServo.setPosition(INTAKE_SERVO_REST);
    }

    // Utility methods
    public boolean isArmAtPosition() {
        return !armMotor.isBusy();
    }
}
