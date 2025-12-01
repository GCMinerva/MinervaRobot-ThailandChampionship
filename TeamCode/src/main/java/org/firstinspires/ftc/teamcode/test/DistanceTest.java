package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Distance Test", group = "Test")
public class DistanceTest extends LinearOpMode {

    private DcMotor intake = null;
    private Servo sorter = null;
    private DistanceSensor distanceSensor = null;

    // Servo positions
    private static final double IN1_POSITION = 0.035;
    private static final double IN2_POSITION = 0.441;
    private static final double IN3_POSITION = 0.782;

    // Distance threshold to trigger sorter (in cm)
    private static final double DISTANCE_THRESHOLD_CM = 4.0;

    private enum SorterPosition {
        IN1,
        IN2,
        IN3
    }

    private SorterPosition currentSorterPosition = SorterPosition.IN1;
    private final ElapsedTime sortTimer = new ElapsedTime();
    private boolean justSorted = false;
    private int in1Passes = 0;

    @Override
    public void runOpMode() {
        // Initialize hardware
        intake = hardwareMap.get(DcMotor.class, "intake");
        sorter = hardwareMap.get(Servo.class, "sorter");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "dis");

        // Set motor direction if necessary (e.g., intake.setDirection(DcMotor.Direction.REVERSE);)

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        sorter.setPosition(IN1_POSITION); // Set to default position at start
        currentSorterPosition = SorterPosition.IN1;
        in1Passes = 1; // We start at IN1

        while (opModeIsActive()) {
            // Set intake power based on sorter position
            if (currentSorterPosition == SorterPosition.IN1 && in1Passes == 2) {
                intake.setPower(0.0); // Stop intake when sorter is at IN1 on the second pass
            } else {
                intake.setPower(-1.0); // Otherwise, run intake
            }

            // Read distance sensor value
            double distance = distanceSensor.getDistance(DistanceUnit.CM);

            if (justSorted && sortTimer.seconds() >= 0.25) {
                justSorted = false;
            }

            // Sorter Logic
            if (!justSorted && distance < DISTANCE_THRESHOLD_CM) {
                switch (currentSorterPosition) {
                    case IN1:
                        sorter.setPosition(IN2_POSITION);
                        currentSorterPosition = SorterPosition.IN2;
                        break;
                    case IN2:
                        sorter.setPosition(IN3_POSITION);
                        currentSorterPosition = SorterPosition.IN3;
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

            // Telemetry for debugging
            telemetry.addData("Distance (cm)", distance);
            telemetry.addData("Sorter Position", currentSorterPosition.toString());
            telemetry.addData("Sorter Servo Position", sorter.getPosition());
            telemetry.addData("Intake Power", intake.getPower());
            telemetry.update();

            // Allow other processes to run
            idle();
        }
    }
}