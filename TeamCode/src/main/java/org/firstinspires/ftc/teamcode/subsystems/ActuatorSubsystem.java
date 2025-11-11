package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.util.GlobalData;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * ActuatorSubsystem — controls shooter wheel (outtake), intake, and servos.
 * Executes different spin patterns depending on detected AprilTag motif (21–23).
 */
public class ActuatorSubsystem {

    private DcMotor outtake, intake;
    private Servo binit, sorter;
    private final Telemetry telemetry;
    private final ElapsedTime timer = new ElapsedTime();

    // === Constructor ===
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

        telemetry.addLine("Actuator Initialized");
        telemetry.update();
    }

    // === Basic control functions ===

    /** Run intake motor to collect game pieces */
    public void runIntake() {
        intake.setPower(-0.4); // Negative to collect
        telemetry.addData("Intake", "Running %.2f", -0.4);
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

    // === Slot Position Sorter Controls ===

    /** Set sorter position for Slot 1 */
    public void setSorterP1() {
        sorter.setPosition(0.3); // Position for Field Position 1
        telemetry.addData("sorter", "Position P1: %.2f", 0.3);
        telemetry.update();
    }

    /** Set sorter position for Slot 2 */
    public void setSorterP2() {
        sorter.setPosition(0.5); // Position for Field Position 2
        telemetry.addData("sorter", "Position P2: %.2f", 0.5);
        telemetry.update();
    }

    /** Set sorter position for Slot 3 */
    public void setSorterP3() {
        sorter.setPosition(0.7); // Position for Field Position 3
        telemetry.addData("sorter", "Position P3: %.2f", 0.7);
        telemetry.update();
    }

    // === Intake Sorter Controls ===

    /** Set sorter position for Intake 1 */
    public void setSorterIN1() {
        sorter.setPosition(0.4); // Position for Intake Position 1
        telemetry.addData("sorter", "Position IN1: %.2f", 0.4);
        telemetry.update();
    }

    /** Set sorter position for Intake 2 */
    public void setSorterIN2() {
        sorter.setPosition(0.6); // Position for Intake Position 2
        telemetry.addData("sorter", "Position IN2: %.2f", 0.6);
        telemetry.update();
    }

    /** Set sorter position for Intake 3 */
    public void setSorterIN3() {
        sorter.setPosition(0.8); // Position for Intake Position 3
        telemetry.addData("sorter", "Position IN3: %.2f", 0.8);
        telemetry.update();
    }

    // === FarGO Function ===
    public void FarGO() {
        int tag = GlobalData.detectedTagId;

        if (tag == 21) {

            setSorterP3();
            sleep(0.4);
            feedBall(1);
            setSorterP2();
            sleep(0.4);
            feedBall(1);
            setSorterP1();
            sleep(0.4);
            feedBall(1);

        } else if (tag == 22) {
            setSorterP1();
            setSorterP3();
            setSorterP2();
        } else if (tag == 23) {
            setSorterP1();
            setSorterP2();
            setSorterP3();
        } else {
            telemetry.addLine("Invalid Tag for FarGO");
            telemetry.update();
        }
    }

    // === MidGO Function ===
    public void MidGO() {
        int tag = GlobalData.detectedTagId;

        if (tag == 21) {
            setSorterP2();
            setSorterP3();
            setSorterP1();
        } else if (tag == 22) {
            setSorterP1();
            setSorterP2();
            setSorterP3();
        } else if (tag == 23) {
            setSorterP1();
            setSorterP3();
            setSorterP2();
        } else {
            telemetry.addLine("Invalid Tag for MidGO");
            telemetry.update();
        }
    }

    // === NearGO Function ===
    public void NearGO() {
        int tag = GlobalData.detectedTagId;

        if (tag == 21) {
            setSorterP1();
            setSorterP2();
            setSorterP3();
        } else if (tag == 22) {
            setSorterP2();
            setSorterP1();
            setSorterP3();
        } else if (tag == 23) {
            setSorterP3();
            setSorterP2();
            setSorterP1();
        } else {
            telemetry.addLine("Invalid Tag for NearGO");
            telemetry.update();
        }
    }

    // === Shooter Control Functions ===

    /** Run shooter for far distance (default high speed) */
    public void runShooterFar() {
        outtake.setPower(1.0); // Max power for far shot
        telemetry.addData("Shooter", "Far shot running at 1.0");
        telemetry.update();
    }

    /** Run shooter for close distance (lower speed) */
    public void runShooterClose() {
        outtake.setPower(0.6); // Lower power for close shot
        telemetry.addData("Shooter", "Close shot running at 0.6");
        telemetry.update();
    }

    /** Stop shooter */
    public void stopShooter() {
        outtake.setPower(0);
        telemetry.addData("Shooter", "Stopped");
        telemetry.update();
    }

    // === Feed Ball Function ===

    /** Feed ball (open and close bin in sequence) */
    public void feedBall(double delaySec) {
        openBin();  // Open bin
        sleep(0.5); // Wait for ball to go through
        closeBin(); // Close bin
        sleep(0.3); // Wait briefly after feeding
    }

    // === Sleep helper (used for waiting) ===
    private void sleep(double seconds) {
        timer.reset();
        while (timer.seconds() < seconds) {
            // simple delay without blocking hardware updates
        }
    }
}
