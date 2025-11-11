package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.util.GlobalData;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Universal actuator subsystem controlling outtake, intake, sorter, and binit.
 * Reads the globally detected AprilTag ID to adjust behavior if needed.
 */
public class ActuatorSubsystem {

    private DcMotor outtake, intake;
    private Servo binit, sorter;
    private final Telemetry telemetry;
    private final ElapsedTime timer = new ElapsedTime();

    public ActuatorSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        outtake = hardwareMap.get(DcMotor.class, "outtake");
        intake = hardwareMap.get(DcMotor.class, "intake");

        outtake.setDirection(DcMotor.Direction.FORWARD);
        outtake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        binit = hardwareMap.get(Servo.class, "binit");
        sorter = hardwareMap.get(Servo.class, "sorter");

        // Default servo positions
        binit.setPosition(0.35);
        sorter.setPosition(0.55);

        telemetry.addData("Actuator", "Initialized");
        telemetry.update();
    }

    // === Universal functions ===

    /** Run intake motor to collect game pieces */
    public void runIntake(double power) {
        intake.setPower(power);
        telemetry.addData("Intake", "Running %.2f", power);
        telemetry.update();
    }

    /** Stop intake */
    public void stopIntake() {
        intake.setPower(0);
        telemetry.addData("Intake", "Stopped");
        telemetry.update();
    }

    /** Run outtake motor */
    public void runOuttake(double power) {
        outtake.setPower(power);
        telemetry.addData("Outtake", "Running %.2f", power);
        telemetry.update();
    }

    /** Stop outtake */
    public void stopOuttake() {
        outtake.setPower(0);
        telemetry.addData("Outtake", "Stopped");
        telemetry.update();
    }

    /** Open bin (binit) */
    public void openBin() {
        binit.setPosition(0.75);
        telemetry.addData("binit", "Opened");
        telemetry.update();
    }

    /** Close bin (binit) */
    public void closeBin() {
        binit.setPosition(0.35);
        telemetry.addData("binit", "Closed");
        telemetry.update();
    }

    /** Move sorter based on direction */
    public void setSorter(double position) {
        sorter.setPosition(position);
        telemetry.addData("sorter", "Position %.2f", position);
        telemetry.update();
    }

    // === Universal function depending on AprilTag ===
    /** Example: adjust starting positions or actions based on AprilTag */
    public void actBasedOnTag() {
        int tag = GlobalData.detectedTagId;

        if (tag == 21) {
            telemetry.addLine("Tag 21: Running Left-Side Routine");
            openBin();
        } else if (tag == 22) {
            telemetry.addLine("Tag 22: Running Center-Side Routine");
            closeBin();
        } else if (tag == 23) {
            telemetry.addLine("Tag 23: Running Right-Side Routine");
            setSorter(0.7);
        } else {
            telemetry.addLine("No valid Tag — Default state");
            stopIntake();
        }
        telemetry.update();
    }
}
