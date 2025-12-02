package org.firstinspires.ftc.teamcode.archived.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.archived.util.GlobalData;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * ActuatorSubsystem — controls shooter wheels (outtake1 & outtake2), intake,
 * and servos.
 * Executes different spin patterns depending on detected AprilTag motif
 * (21–23).
 */
public class ActuatorSubsystem {

    private DcMotor outtake1, outtake2, intake;
    private Servo binit, brinit, sorter;
    private DistanceSensor distanceSensor;
    private final Telemetry telemetry;
    private final ElapsedTime timer = new ElapsedTime();

    // Ball detection variables
    private final ElapsedTime sortTimer = new ElapsedTime();
    private boolean justSorted = false;
    private int ballCount = 0;
    private int in1Passes = 0;
    private static final double DISTANCE_THRESHOLD_CM = 4.0;
    private static final double SORT_DEBOUNCE_TIME = 0.25; // seconds

    // Sorter position tracking
    private enum SorterPosition {
        IN1, IN2, IN3, OTHER
    }

    private SorterPosition currentSorterPosition = SorterPosition.OTHER;

    // === Constructor ===
    public ActuatorSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        // Initialize motors
        outtake1 = hardwareMap.get(DcMotor.class, "outtake1");
        outtake2 = hardwareMap.get(DcMotor.class, "outtake2");
        intake = hardwareMap.get(DcMotor.class, "intake");

        // Set motor directions and behaviors
        outtake1.setDirection(DcMotor.Direction.REVERSE);
        outtake2.setDirection(DcMotor.Direction.REVERSE);
        outtake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize servos
        binit = hardwareMap.get(Servo.class, "binit");
        brinit = hardwareMap.get(Servo.class, "brinit");
        sorter = hardwareMap.get(Servo.class, "sorter");

        // Initialize distance sensor (optional - may not be present)
        try {
            distanceSensor = hardwareMap.get(DistanceSensor.class, "dis");
        } catch (Exception e) {
            distanceSensor = null;
            // telemetry.addLine("Warning: Distance sensor not found");
        }

        // Default servo positions
        binit.setPosition(0.1);
        brinit.setPosition(0.8);
        sorter.setPosition(0.441);

        // telemetry.addLine("Actuator Initialized");
        // telemetry.update();
    }

    // === Basic control functions ===

    /** Run intake motor to collect game pieces */
    public void runIntake() {
        intake.setPower(-1); // Negative to collect
        // telemetry.addData("Intake", "Running %.2f", -1);
        // telemetry.update();
    }

    /** Stop intake */
    public void stopIntake() {
        intake.setPower(0);
        // telemetry.addData("Intake", "Stopped");
        // telemetry.update();
    }

    /** Set intake power */
    public void setIntakePower(double power) {
        intake.setPower(power);
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
        sorter.setPosition(0.592); // Position for Field Position 1
        // telemetry.addData("sorter", "Position P1: %.2f", 0.751);
        // telemetry.update();
    }

    /** Set sorter position for Slot 2 */
    public void setSorterP2() {
        sorter.setPosition(0.979); // Position for Field Position 2
        // telemetry.addData("sorter", "Position P2: %.2f", 0);
        // telemetry.update();
    }

    /** Set sorter position for Slot 3 */
    public void setSorterP3() {
        sorter.setPosition(0.219); // Position for Field Position 3
        // telemetry.addData("sorter", "Position P3: %.2f", 0.354);
        // telemetry.update();
    }

    // === Intake Sorter Controls ===

    /** Set sorter position for Intake 1 */
    public void setSorterIN1() {
        sorter.setPosition(0.035); // Position for Intake Position 1
        currentSorterPosition = SorterPosition.IN1;
        // telemetry.addData("sorter", "Position IN1: %.2f", 0.15);
        // telemetry.update();
    }

    /** Set sorter position for Intake 2 */
    public void setSorterIN2() {
        sorter.setPosition(0.406); // Position for Intake Position 2 (updated from 0.41)
        currentSorterPosition = SorterPosition.IN2;
        // telemetry.addData("sorter", "Position IN2: %.2f", 0.533);
        // telemetry.update();
    }

    /** Set sorter position for Intake 3 */
    public void setSorterIN3() {
        sorter.setPosition(0.782); // Position for Intake Position 3
        currentSorterPosition = SorterPosition.IN3;
        // telemetry.addData("sorter", "Position IN3: %.2f", 0.918);
        // telemetry.update();
    }

    // === Automatic Ball Detection and Sorting ===

    /**
     * Initialize ball detection system
     */
    public void initBallDetection() {
        ballCount = 0;
        in1Passes = 1;
        justSorted = false;
        sortTimer.reset();
        setSorterIN1();
    }

    /**
     * Update ball detection - call this periodically in a loop
     * 
     * @return true if still collecting, false if 3+ balls collected
     */
    public boolean updateBallDetection() {
        if (distanceSensor == null) {
            return true; // No sensor, continue normally
        }

        // Read distance sensor
        double distance = distanceSensor.getDistance(DistanceUnit.CM);

        // Reset debounce flag after timeout
        if (justSorted && sortTimer.seconds() >= SORT_DEBOUNCE_TIME) {
            justSorted = false;
        }

        // Detect ball and advance sorter
        if (!justSorted && distance < DISTANCE_THRESHOLD_CM) {
            ballCount++;

            switch (currentSorterPosition) {
                case IN1:
                    setSorterIN2();
                    break;
                case IN2:
                    setSorterIN3();
                    break;
                case IN3:
                    setSorterIN1();
                    in1Passes++;
                    break;
                default:
                    // If position is unknown, reset to IN1
                    setSorterIN1();
                    break;
            }

            sortTimer.reset();
            justSorted = true;
        }

        return true; // Continue collecting
    }

    /**
     * Get current ball count
     */
    public int getBallCount() {
        return ballCount;
    }

    /**
     * Check if 3 or more balls have been collected
     */
    public boolean isBallsFull() {
        return ballCount >= 3;
    }

    /**
     * Reset ball count
     */
    public void resetBallCount() {
        ballCount = 0;
    }

    // === JUST SHOOT ===
    public void justShoot() {

        int tag = GlobalData.detectedTagId;

        setSorterP3();
        sleep(0.2); // Wait for sorter to move to P3
        feedBall(0.3); // Feed ball (non-blocking)
        sleep(0.45); // Wait before next position
        outtake1.setPower(-0.64);

        setSorterP1();
        sleep(0.33); // Medium wait - P3 to P2 distance
        feedBall(0.3); // Feed ball (non-blocking)
        sleep(0.45); // Wait before next position
        outtake1.setPower(-0.8);

        setSorterP2();
        sleep(0.3); // Longer wait - P2 to P1 is farther distance
        feedBall(0.3); // Feed ball (non-blocking)
        sleep(0.1);
        // telemetry.addLine("Invalid Tag for Just shoot");
        // telemetry.update();

    }

    // === FarGO Function ===
    public void FarGOready() {

        int tag = GlobalData.detectedTagId;
        if (tag == 21) {

            setSorterP3();

        } else if (tag == 22) {
            setSorterP1();

        } else if (tag == 23) {
            setSorterP1();

        } else {

            setSorterP3();

        }

    }

    public void FarGO() {
        int tag = GlobalData.detectedTagId;

        if (tag == 21) {
            sleep(0.1);
            feedBall(0.3); // Feed ball (non-blocking)
            sleep(0.45); // Wait before next position

            setSorterP2();
            sleep(0.62); // Medium wait - P3 to P2 distance
            feedBall(0.3); // Feed ball (non-blocking)
            sleep(0.45); // Wait before next position
            outtake1.setPower(-0.64);

            setSorterP1();
            sleep(0.3); // Longer wait - P2 to P1 is farther distance
            feedBall(0.3);

        } else if (tag == 22) {
            sleep(0.1);
            feedBall(0.3); // Feed ball (non-blocking)
            sleep(0.45);
            outtake1.setPower(-0.64);

            setSorterP3();
            sleep(0.27); // Wait for sorter to move to P3
            feedBall(0.3); // Feed ball (non-blocking)
            sleep(0.45);
            outtake1.setPower(-0.61);

            setSorterP2();
            sleep(0.6); // Wait for sorter to move to P3
            feedBall(0.3); // Feed ball (non-blocking)
            sleep(0.45);

        } else if (tag == 23) {

            sleep(0.1);
            feedBall(0.3); // Feed ball (non-blocking)
            sleep(0.45);
            outtake1.setPower(-0.65);

            setSorterP2();
            sleep(0.3); // Wait for sorter to move to P3
            feedBall(0.3); // Feed ball (non-blocking)
            sleep(0.45);
            outtake1.setPower(-0.61);

            setSorterP3();
            sleep(0.6); // Wait for sorter to move to P3
            feedBall(0.3); // Feed ball (non-blocking)
            sleep(0.45);

        } else {
            sleep(0.1);
            feedBall(0.3); // Feed ball (non-blocking)
            sleep(0.45); // Wait before next position

            setSorterP1();
            sleep(0.3); // Medium wait - P3 to P2 distance
            feedBall(0.3); // Feed ball (non-blocking)
            sleep(0.45); // Wait before next position

            setSorterP2();
            sleep(0.3); // Longer wait - P2 to P1 is farther distance
            feedBall(0.3);
        }
    }

    // === MidGO Function ===
    public void MidGoready() {

        int tag = GlobalData.detectedTagId;
        if (tag == 21) {

            setSorterP2();

        } else if (tag == 22) {
            setSorterP1();

        } else if (tag == 23) {
            setSorterP1();

        } else {

            setSorterP3();

        }

    }

    public void MidGO() {
        int tag = GlobalData.detectedTagId;

        if (tag == 21) {
            sleep(0.1);
            feedBall(0.3); // Feed ball (non-blocking)
            sleep(0.45); // Wait before next position
            outtake1.setPower(-0.61);

            setSorterP3();
            sleep(0.6); // Wait for sorter to move to P3
            feedBall(0.3); // Feed ball (non-blocking)
            sleep(0.45);

            setSorterP1();
            sleep(0.5); // Wait for sorter to move to P3
            feedBall(0.3); // Feed ball (non-blocking)
            sleep(0.45);

        } else if (tag == 22) {
            sleep(0.1);
            feedBall(0.3); // Feed ball (non-blocking)
            sleep(0.45); // Wait before next position

            setSorterP2();
            sleep(0.5); // Wait for sorter to move to P3
            feedBall(0.3); // Feed ball (non-blocking)
            sleep(0.45);

            setSorterP3();
            sleep(0.6); // Wait for sorter to move to P3
            feedBall(0.3); // Feed ball (non-blocking)
            sleep(0.45);

        } else if (tag == 23) {
            sleep(0.1);
            feedBall(0.3); // Feed ball (non-blocking)
            sleep(0.45); // Wait before next position
            outtake1.setPower(-0.65);

            setSorterP3();
            sleep(0.3); // Wait for sorter to move to P3
            feedBall(0.3); // Feed ball (non-blocking)
            sleep(0.45);
            outtake1.setPower(-0.61);

            setSorterP2();
            sleep(0.6); // Wait for sorter to move to P3
            feedBall(0.3); // Feed ball (non-blocking)
            sleep(0.45);

        } else {
            // telemetry.addLine("Invalid Tag for MidGO");
            // telemetry.update();
            sleep(0.1);
            feedBall(0.3); // Feed ball (non-blocking)
            sleep(0.45); // Wait before next position

            setSorterP1();
            sleep(0.3); // Medium wait - P3 to P2 distance
            feedBall(0.3); // Feed ball (non-blocking)
            sleep(0.45); // Wait before next position

            setSorterP2();
            sleep(0.3); // Longer wait - P2 to P1 is farther distance
            feedBall(0.3);
        }
    }

    // === NearGO Function ===
    public void NearGOready() {

        int tag = GlobalData.detectedTagId;
        if (tag == 21) {

            setSorterP1();

        } else if (tag == 22) {
            setSorterP2();

        } else if (tag == 23) {
            setSorterP3();

        } else {

            setSorterP3();

        }

    }

    public void NearGO() {
        int tag = GlobalData.detectedTagId;

        if (tag == 21) {
            sleep(0.1);
            feedBall(0.3); // Feed ball (non-blocking)
            sleep(0.45);
            outtake1.setPower(-0.65);

            setSorterP2();
            sleep(0.32); // Wait for sorter to move to P3
            feedBall(0.3); // Feed ball (non-blocking)
            sleep(0.45);
            outtake1.setPower(-0.62);

            setSorterP3();
            sleep(0.44); // Wait for sorter to move to P3
            feedBall(0.3); // Feed ball (non-blocking)
//            sleep(0.45);

        } else if (tag == 22) {
            sleep(0.1);
            feedBall(0.3); // Feed ball (non-blocking)
            sleep(0.45);
            outtake1.setPower(-0.65);

            setSorterP1();
            sleep(0.3); // Wait for sorter to move to P3
            feedBall(0.3); // Feed ball (non-blocking)
            sleep(0.45);
            outtake1.setPower(-0.66);

            setSorterP3();
            sleep(0.25); // Wait for sorter to move to P3
            feedBall(0.3); // Feed ball (non-blocking)
//            sleep(0.45);

        } else if (tag == 23) {
            sleep(0.1);
            feedBall(0.3); // Feed ball (non-blocking)
            sleep(0.45);
            outtake1.setPower(-0.60);

            setSorterP2();
            sleep(0.63); // Wait for sorter to move to P3
            feedBall(0.3); // Feed ball (non-blocking)
            sleep(0.45);
            outtake1.setPower(-0.65);

            setSorterP1();
            sleep(0.3); // Wait for sorter to move to P3
            feedBall(0.3); // Feed ball (non-blocking)

        } else {
            sleep(0.1);
            feedBall(0.3); // Feed ball (non-blocking)
            sleep(0.45); // Wait before next position

            setSorterP2();
            sleep(0.3); // Medium wait - P3 to P2 distance
            feedBall(0.3); // Feed ball (non-blocking)
            sleep(0.45); // Wait before next position

            setSorterP1();
            sleep(0.5); // Longer wait - P2 to P1 is farther distance
            feedBall(0.3);
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
        outtake1.setPower(-0.595); // Lower power for close shot
        outtake2.setPower(-0.595); // Lower power for close shot
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
                Thread.sleep((long) (delaySec * 1000)); // Wait specified time
                openBin(); // Reopen bin
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }).start();
    }

    // === Sleep helper (used for waiting without blocking callbacks) ===
    private void sleep(double seconds) {
        try {
            Thread.sleep((long) (seconds * 1000)); // Convert to milliseconds
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}
