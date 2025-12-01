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
    private static final double DISTANCE_THRESHOLD_CM = 5.0;

    private enum SorterState {
        IDLE,
        SORTING_STEP_1,
        SORTING_STEP_2,
        SORTING_STEP_3
    }

    private SorterState sorterState = SorterState.IDLE;
    private ElapsedTime sortTimer = new ElapsedTime();

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
        
        while (opModeIsActive()) {
            // Set intake power
            intake.setPower(-1.0);

            // Read distance sensor value
            double distance = distanceSensor.getDistance(DistanceUnit.CM);

            // Sorter State Machine
            switch (sorterState) {
                case IDLE:
                    // If we detect an object close enough, start the sorting sequence.
                    if (distance < DISTANCE_THRESHOLD_CM) {
                        sortTimer.reset();
                        sorterState = SorterState.SORTING_STEP_1;
                    }
                    break;

                case SORTING_STEP_1:
                    sorter.setPosition(IN2_POSITION);
                    if (sortTimer.milliseconds() > 250) {
                        sortTimer.reset();
                        sorterState = SorterState.SORTING_STEP_2;
                    }
                    break;

                case SORTING_STEP_2:
                    sorter.setPosition(IN3_POSITION);
                    if (sortTimer.milliseconds() > 250) {
                        sortTimer.reset();
                        sorterState = SorterState.SORTING_STEP_3;
                    }
                    break;

                case SORTING_STEP_3:
                    sorter.setPosition(IN1_POSITION); // Return to default
                    if (sortTimer.milliseconds() > 250) {
                        sorterState = SorterState.IDLE; // End of sequence
                    }
                    break;
            }

            // Telemetry for debugging
            telemetry.addData("Distance (cm)", distance);
            telemetry.addData("Sorter State", sorterState.toString());
            telemetry.addData("Sorter Position", sorter.getPosition());
            telemetry.addData("Intake Power", intake.getPower());
            telemetry.update();

            // Allow other processes to run
            idle();
        }
    }
}
