package org.firstinspires.ftc.teamcode.archived.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.archived.util.GlobalData;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * ActuatorSubsystem — controls shooter wheels (outtake1 & outtake2), intake, and servos.
 * Executes different spin patterns depending on detected AprilTag motif (21–23).
 */
public class ActuatorSubsystem {

    private DcMotor outtake1, outtake2, intake;
    private Servo binit, brinit, sorter;
    private final Telemetry telemetry;
    private final ElapsedTime timer = new ElapsedTime();

    // === Constructor ===
    public ActuatorSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        // Initialize motors
        outtake1 = hardwareMap.get(DcMotor.class, "outtake1");
        outtake2 = hardwareMap.get(DcMotor.class, "outtake2");
        intake = hardwareMap.get(DcMotor.class, "intake");

        // Set motor directions and behaviors
        outtake1.setDirection(DcMotor.Direction.FORWARD);
        outtake2.setDirection(DcMotor.Direction.REVERSE);
        outtake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize servos
        binit = hardwareMap.get(Servo.class, "binit");
        brinit = hardwareMap.get(Servo.class, "brinit");
        sorter = hardwareMap.get(Servo.class, "sorter");

        // Default servo positions
        binit.setPosition(0.35);
        sorter.setPosition(0.55);

        // telemetry.addLine("Actuator Initialized");
        // telemetry.update();
    }

    // === Basic control functions ===

    /** Run intake motor to collect game pieces */
    public void runIntake() {
        intake.setPower(-0.93); // Negative to collect
        // telemetry.addData("Intake", "Running %.2f", -1);
        // telemetry.update();
    }

    /** Stop intake */
    public void stopIntake() {
        intake.setPower(0);
        // telemetry.addData("Intake", "Stopped");
        // telemetry.update();
    }

    /** Run outtake motors (both at the same time) */
    public void runOuttake(double power) {
        outtake1.setPower(power);
        outtake2.setPower(power); // Set both motors to the same power
        // telemetry.addData("Outtake", "Running %.2f", power);
        // telemetry.update();
    }

    /** Stop outtake motors */
    public void stopOuttake() {
        outtake1.setPower(0);
        outtake2.setPower(0);
        // telemetry.addData("Outtake", "Stopped");
        // telemetry.update();
    }

    /** Open bin (binit) */
    public void openBin() {
        binit.setPosition(0.1);
        brinit.setPosition(0.8);
        // telemetry.addData("binit", "Opened");
        // telemetry.update();
    }

    /** Close bin (binit) */
    public void closeBin() {
        binit.setPosition(0.56);
        brinit.setPosition(0.34);
        // telemetry.addData("binit", "Closed");
        // telemetry.update();
    }

    /** Move sorter based on direction */
    public void setSorter(double position) {
        sorter.setPosition(position);
        // telemetry.addData("sorter", "Position %.2f", position);
        // telemetry.update();
    }

    // === Slot Position Sorter Controls ===

    /** Set sorter position for Slot 1 */
    public void setSorterP1() {
        sorter.setPosition(0.751); // Position for Field Position 1
        // telemetry.addData("sorter", "Position P1: %.2f", 0.751);
        // telemetry.update();
    }

    /** Set sorter position for Slot 2 */
    public void setSorterP2() {
        sorter.setPosition(0); // Position for Field Position 2
        // telemetry.addData("sorter", "Position P2: %.2f", 0);
        // telemetry.update();
    }

    /** Set sorter position for Slot 3 */
    public void setSorterP3() {
        sorter.setPosition(0.354); // Position for Field Position 3
        // telemetry.addData("sorter", "Position P3: %.2f", 0.354);
        // telemetry.update();
    }

    // === Intake Sorter Controls ===

    /** Set sorter position for Intake 1 */
    public void setSorterIN1() {
        sorter.setPosition(0.157); // Position for Intake Position 1
        // telemetry.addData("sorter", "Position IN1: %.2f", 0.15);
        // telemetry.update();
    }

    /** Set sorter position for Intake 2 */
    public void setSorterIN2() {
        sorter.setPosition(0.533); // Position for Intake Position 2
        // telemetry.addData("sorter", "Position IN2: %.2f", 0.533);
        // telemetry.update();
    }

    /** Set sorter position for Intake 3 */
    public void setSorterIN3() {
        sorter.setPosition(0.918); // Position for Intake Position 3
        // telemetry.addData("sorter", "Position IN3: %.2f", 0.918);
        // telemetry.update();
    }

    // === JUST SHOOT ===
    public void justShoot() {

        int tag = GlobalData.detectedTagId;

        if (tag == 21) {

            setSorterP3();
            sleep(0.25);      // Wait for sorter to move to P3
            feedBall(0.5);   // Feed ball (non-blocking)
            sleep(1.1);      // Wait before next position

            setSorterP2();
            sleep(0.3);      // Medium wait - P3 to P2 distance
            feedBall(0.5);   // Feed ball (non-blocking)
            sleep(1.1);      // Wait before next position

            setSorterP1();
            sleep(0.5);      // Longer wait - P2 to P1 is farther distance
            feedBall(0.5);

        } else if (tag == 22) {
            setSorterP1();
            sleep(0.4);      // Wait for sorter to move to P3
            feedBall(0.5);   // Feed ball (non-blocking)
            sleep(1.1);

            setSorterP3();
            sleep(0.2);      // Wait for sorter to move to P3
            feedBall(0.5);   // Feed ball (non-blocking)
            sleep(1.1);

            setSorterP2();
            sleep(0.2);      // Wait for sorter to move to P3
            feedBall(0.5);   // Feed ball (non-blocking)
            sleep(1.1);

        } else if (tag == 23) {
            setSorterP1();
            sleep(0.35);      // Wait for sorter to move to P3
            feedBall(0.5);   // Feed ball (non-blocking)
            sleep(1.1);

            setSorterP2();
            sleep(0.2);      // Wait for sorter to move to P3
            feedBall(0.5);   // Feed ball (non-blocking)
            sleep(1.1);

            setSorterP3();
            sleep(0.2);      // Wait for sorter to move to P3
            feedBall(0.5);   // Feed ball (non-blocking)
            sleep(1.1);

        } else {
            setSorterP3();
            sleep(0.2);      // Wait for sorter to move to P3
            feedBall(0.5);   // Feed ball (non-blocking)
            sleep(1.1);      // Wait before next position
            
            setSorterP2();
            sleep(0.3);      // Medium wait - P3 to P2 distance
            feedBall(0.5);   // Feed ball (non-blocking)
            sleep(1.1);      // Wait before next position
            
            setSorterP1();
            sleep(0.5);      // Longer wait - P2 to P1 is farther distance
            feedBall(0.5);   // Feed ball (non-blocking)
            // telemetry.addLine("Invalid Tag for Just shoot");
            // telemetry.update();
        }
    }

    // === FarGO Function ===
    public void FarGO() {
        int tag = GlobalData.detectedTagId;

        if (tag == 21) {

            setSorterP3();
            sleep(0.45);      // Wait for sorter to move to P3
            feedBall(1);   // Feed ball (non-blocking)
            sleep(1.1);      // Wait before next position

            setSorterP2();
            sleep(0.3);      // Medium wait - P3 to P2 distance
            feedBall(1);   // Feed ball (non-blocking)
            sleep(1.1);      // Wait before next position

            setSorterP1();
            sleep(0.5);      // Longer wait - P2 to P1 is farther distance
            feedBall(1);

        } else if (tag == 22) {
            setSorterP1();
            sleep(0.2);      // Wait for sorter to move to P3
            feedBall(0.5);   // Feed ball (non-blocking)
            sleep(1.1);

            setSorterP3();
            sleep(0.3);      // Wait for sorter to move to P3
            feedBall(0.5);   // Feed ball (non-blocking)
            sleep(1.1);

            setSorterP2();
            sleep(0.25);      // Wait for sorter to move to P3
            feedBall(0.5);   // Feed ball (non-blocking)
            sleep(1.1);

        } else if (tag == 23) {
            setSorterP1();
            sleep(0.2);      // Wait for sorter to move to P3
            feedBall(0.5);   // Feed ball (non-blocking)
            sleep(1.1);

            setSorterP2();
            sleep(0.5);      // Wait for sorter to move to P3
            feedBall(0.5);   // Feed ball (non-blocking)
            sleep(1.1);

            setSorterP3();
            sleep(0.25);      // Wait for sorter to move to P3
            feedBall(0.5);   // Feed ball (non-blocking)
            sleep(1.1);

        } else {
            // telemetry.addLine("Invalid Tag for FarGO");
            // telemetry.update();
            setSorterP3();
            sleep(0.45);      // Wait for sorter to move to P3
            feedBall(1);   // Feed ball (non-blocking)
            sleep(1.1);      // Wait before next position

            setSorterP2();
            sleep(0.3);      // Medium wait - P3 to P2 distance
            feedBall(1);   // Feed ball (non-blocking)
            sleep(1.1);      // Wait before next position

            setSorterP1();
            sleep(0.5);      // Longer wait - P2 to P1 is farther distance
            feedBall(1);   // Feed ball (non-blocking)
            // telemetry.addLine("Invalid Tag for Just shoot");
            // telemetry.update();
        }
    }

    // === MidGO Function ===
    public void MidGO() {
        int tag = GlobalData.detectedTagId;

        if (tag == 21) {
            setSorterP2();
            sleep(0.5);      // Wait for sorter to move to P3
            feedBall(1);   // Feed ball (non-blocking)
            sleep(1.1);      // Wait before next position

            setSorterP3();
            sleep(0.25);      // Wait for sorter to move to P3
            feedBall(0.5);   // Feed ball (non-blocking)
            sleep(1.1);

            setSorterP1();
            sleep(0.5);      // Wait for sorter to move to P3
            feedBall(0.5);   // Feed ball (non-blocking)
            sleep(1.1);

        } else if (tag == 22) {
            setSorterP1();
            sleep(0.25);      // Wait for sorter to move to P3
            feedBall(1);   // Feed ball (non-blocking)
            sleep(1.1);      // Wait before next position

            setSorterP2();
            sleep(0.5);      // Wait for sorter to move to P3
            feedBall(0.5);   // Feed ball (non-blocking)
            sleep(1.1);

            setSorterP3();
            sleep(0.23);      // Wait for sorter to move to P3
            feedBall(0.5);   // Feed ball (non-blocking)
            sleep(1.1);

        } else if (tag == 23) {
            setSorterP1();
            sleep(0.25);      // Wait for sorter to move to P3
            feedBall(1);   // Feed ball (non-blocking)
            sleep(1.1);      // Wait before next position

            setSorterP3();
            sleep(0.25);      // Wait for sorter to move to P3
            feedBall(0.5);   // Feed ball (non-blocking)
            sleep(1.1);

            setSorterP2();
            sleep(0.25);      // Wait for sorter to move to P3
            feedBall(0.5);   // Feed ball (non-blocking)
            sleep(1.1);

        } else {
            // telemetry.addLine("Invalid Tag for MidGO");
            // telemetry.update();
            setSorterP3();
            sleep(0.45);      // Wait for sorter to move to P3
            feedBall(1);   // Feed ball (non-blocking)
            sleep(1.1);      // Wait before next position

            setSorterP2();
            sleep(0.3);      // Medium wait - P3 to P2 distance
            feedBall(1);   // Feed ball (non-blocking)
            sleep(1.1);      // Wait before next position

            setSorterP1();
            sleep(0.5);      // Longer wait - P2 to P1 is farther distance
            feedBall(1);
        }
    }

    // === NearGO Function ===
    public void NearGO() {
        int tag = GlobalData.detectedTagId;

        if (tag == 21) {
            setSorterP1();
            sleep(0.25);      // Wait for sorter to move to P3
            feedBall(0.5);   // Feed ball (non-blocking)
            sleep(1.1);

            setSorterP2();
            sleep(0.5);      // Wait for sorter to move to P3
            feedBall(0.5);   // Feed ball (non-blocking)
            sleep(1.1);

            setSorterP3();
            sleep(0.25);      // Wait for sorter to move to P3
            feedBall(0.5);   // Feed ball (non-blocking)
            sleep(1.1);

        } else if (tag == 22) {
            setSorterP2();
            sleep(0.5);      // Wait for sorter to move to P3
            feedBall(0.5);   // Feed ball (non-blocking)
            sleep(1.1);

            setSorterP1();
            sleep(0.5);      // Wait for sorter to move to P3
            feedBall(0.5);   // Feed ball (non-blocking)
            sleep(1.1);

            setSorterP3();
            sleep(0.25);      // Wait for sorter to move to P3
            feedBall(0.5);   // Feed ball (non-blocking)
            sleep(1.1);

        } else if (tag == 23) {
            setSorterP3();
            sleep(0.45);      // Wait for sorter to move to P3
            feedBall(0.5);   // Feed ball (non-blocking)
            sleep(1.1);

            setSorterP2();
            sleep(0.25);      // Wait for sorter to move to P3
            feedBall(0.5);   // Feed ball (non-blocking)
            sleep(1.1);

            setSorterP1();
            sleep(0.5);      // Wait for sorter to move to P3
            feedBall(0.5);   // Feed ball (non-blocking)
            sleep(1.1);
        } else {
            setSorterP3();
            sleep(0.45);      // Wait for sorter to move to P3
            feedBall(0.5);   // Feed ball (non-blocking)
            sleep(1.1);      // Wait before next position

            setSorterP2();
            sleep(0.3);      // Medium wait - P3 to P2 distance
            feedBall(0.5);   // Feed ball (non-blocking)
            sleep(1.1);      // Wait before next position

            setSorterP1();
            sleep(0.5);      // Longer wait - P2 to P1 is farther distance
            feedBall(0.5);
            // telemetry.addLine("Invalid Tag for NearGO");
            // telemetry.update();
        }
    }

    // === Shooter Control Functions ===

    /** Run shooter for far distance (default high speed) */
    public void runShooterFar() {
        outtake1.setPower(-0.8); // Max power for far shot
        outtake2.setPower(-0.8); // Max power for far shot
        // telemetry.addData("Shooter", "Far shot running at 1.0");
        // telemetry.update();
    }

    /** Run shooter for close distance (lower speed) */
    public void runShooterClose() {
        outtake1.setPower(-0.59); // Lower power for close shot
        outtake2.setPower(-0.59); // Lower power for close shot
        // telemetry.addData("Shooter", "Close shot running at 0.6");
        // telemetry.update();
    }

    /** Stop shooter */
    public void stopShooter() {
        outtake1.setPower(0);
        outtake2.setPower(0);
        // telemetry.addData("Shooter", "Stopped");
        // telemetry.update();
    }

    // === Feed Ball Function ===

    /** Feed ball (open and close bin in sequence) - non-blocking */
    public void feedBall(double delaySec) {
        // Close bin first, then reopen after delay in separate thread
        closeBin();
        new Thread(() -> {
            try {
                Thread.sleep((long)(delaySec * 1000));  // Wait specified time
                openBin();  // Reopen bin
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }).start();
    }

    // === Sleep helper (used for waiting without blocking callbacks) ===
    private void sleep(double seconds) {
        try {
            Thread.sleep((long)(seconds * 1000));  // Convert to milliseconds
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}
