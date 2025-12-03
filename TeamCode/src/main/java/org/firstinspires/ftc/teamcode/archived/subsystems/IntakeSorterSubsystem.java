package org.firstinspires.ftc.teamcode.archived.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * IntakeSorterSubsystem - controls intake motor, sorter servo, and distance
 * sensor
 * for automatic ball collection and sorting.
 * 
 * This subsystem can collect up to 3 balls and automatically stop when the
 * limit is reached.
 */
public class IntakeSorterSubsystem {

    // Hardware components
    private DcMotor intake;
    private Servo sorter;
    private DistanceSensor distanceSensor;
    private Telemetry telemetry;

    // Servo positions for the three intake slots
    private static final double IN1_POSITION = 0.035;
    private static final double IN2_POSITION = 0.406;
    private static final double IN3_POSITION = 0.782;

    // Distance threshold to trigger sorter (in cm)
    private static final double DISTANCE_THRESHOLD_CM = 4.0;

    // Sorter debounce timer (prevents multiple triggers from same ball)
    private static final double SORT_DEBOUNCE_TIME = 0.3; // seconds

    // Sorter position enum
    private enum SorterPosition {
        IN1,
        IN2,
        IN3
    }

    // State variables
    private SorterPosition currentSorterPosition = SorterPosition.IN1;
    private final ElapsedTime sortTimer = new ElapsedTime();
    private boolean justSorted = false;
    private int ballCount = 0;
    private int in1Passes = 0;

    /**
     * Constructor - initializes the subsystem with hardware map and telemetry
     */
    public IntakeSorterSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        // Initialize hardware
        intake = hardwareMap.get(DcMotor.class, "intake");
        sorter = hardwareMap.get(Servo.class, "sorter");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "dis");
    }

    /**
     * Initialize the subsystem - sets sorter to starting position
     */
    public void init() {
        sorter.setPosition(IN1_POSITION);
        currentSorterPosition = SorterPosition.IN1;
        ballCount = 0;
        in1Passes = 1; // We start at IN1
        justSorted = false;
        sortTimer.reset();
    }

    /**
     * Periodic update - call this in a loop to handle ball detection and sorting
     * 
     * @return true if still collecting balls (< 3), false if 3 or more balls
     *         collected
     */
    public boolean update() {
        // Check if we've reached the ball limit
        if (ballCount >= 3) {
            stopIntake();
            return false; // Signal that we're done collecting
        }

        // Set intake power based on sorter position
        if (currentSorterPosition == SorterPosition.IN1 && in1Passes == 2) {
            intake.setPower(0.0); // Stop intake when sorter is at IN1 on the second pass
        } else {
            intake.setPower(-1.0); // Otherwise, run intake
        }

        // Read distance sensor value
        double distance = distanceSensor.getDistance(DistanceUnit.CM);

        // Reset debounce flag after timeout
        if (justSorted && sortTimer.seconds() >= SORT_DEBOUNCE_TIME) {
            justSorted = false;
        }

        // Sorter Logic - detect ball and advance sorter position
        if (!justSorted && distance < DISTANCE_THRESHOLD_CM) {
            ballCount++; // Increment ball count when a ball is detected

            switch (currentSorterPosition) {
                case IN1:
                    sorter.setPosition(IN2_POSITION);
                    currentSorterPosition = SorterPosition.IN2;
                    break;
                case IN2:
                    sorter.setPosition(IN3_POSITION);
                    currentSorterPosition = SorterPosition.IN3;

                    // Check if we've reached the ball limit
                    if (ballCount >= 3) {
                        stopIntake();
                        return false; // Exit and finish
                    }
                    break;
                case IN3:
                    sorter.setPosition(IN1_POSITION);
                    currentSorterPosition = SorterPosition.IN1;
                    in1Passes++; // a pass is when we return to IN1
                    break;
            }
            sortTimer.reset();
            justSorted = true;
        }

        // Update telemetry
        updateTelemetry(distance);

        return true; // Continue collecting
    }

    /**
     * Collect balls until 3 or more are collected
     * This is a blocking function that will run until the ball limit is reached
     * 
     * @param opModeActive Supplier function that returns whether the OpMode is
     *                     still active
     */
    public void collectBallsUntilFull(java.util.function.BooleanSupplier opModeActive) {
        init(); // Reset state

        while (opModeActive.getAsBoolean() && update()) {
            // update() returns false when 3 balls are collected
            try {
                Thread.sleep(10); // Small delay to prevent CPU hogging
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                break;
            }
        }

        stopIntake(); // Ensure intake is stopped when done
    }

    /**
     * Run intake motor
     */
    public void runIntake() {
        intake.setPower(-1.0);
    }

    /**
     * Run intake motor at specified power
     */
    public void runIntake(double power) {
        intake.setPower(power);
    }

    /**
     * Stop intake motor
     */
    public void stopIntake() {
        intake.setPower(0.0);
    }

    /**
     * Set sorter to IN1 position
     */
    public void setSorterIN1() {
        sorter.setPosition(IN1_POSITION);
        currentSorterPosition = SorterPosition.IN1;
    }

    /**
     * Set sorter to IN2 position
     */
    public void setSorterIN2() {
        sorter.setPosition(IN2_POSITION);
        currentSorterPosition = SorterPosition.IN2;
    }

    /**
     * Set sorter to IN3 position
     */
    public void setSorterIN3() {
        sorter.setPosition(IN3_POSITION);
        currentSorterPosition = SorterPosition.IN3;
    }

    /**
     * Get current ball count
     */
    public int getBallCount() {
        return ballCount;
    }

    /**
     * Reset ball count
     */
    public void resetBallCount() {
        ballCount = 0;
    }

    /**
     * Check if ball limit (3) has been reached
     */
    public boolean isFull() {
        return ballCount >= 3;
    }

    /**
     * Get current distance sensor reading in cm
     */
    public double getDistance() {
        return distanceSensor.getDistance(DistanceUnit.CM);
    }

    /**
     * Update telemetry with current state
     */
    private void updateTelemetry(double distance) {
        telemetry.addData("Distance (cm)", distance);
        telemetry.addData("Sorter Position", currentSorterPosition.toString());
        telemetry.addData("Sorter Servo Position", sorter.getPosition());
        telemetry.addData("Intake Power", intake.getPower());
        telemetry.addData("Ball Count", ballCount);
    }

    /**
     * Force update telemetry (call telemetry.update())
     */
    public void updateTelemetryNow() {
        updateTelemetry(getDistance());
        telemetry.update();
    }
}
