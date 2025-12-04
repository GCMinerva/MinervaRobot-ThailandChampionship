package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
@Disabled
@TeleOp(name = "DkTestBall", group = "Test")
public class DkTestBall extends LinearOpMode {

    private DcMotor intake = null;
    private Servo sorter = null;
    private ColorSensor colorSensor = null;

    // Servo positions
    private static final double IN1_POSITION = 0.035;
    private static final double IN2_POSITION = 0.441;
    private static final double IN3_POSITION = 0.782;

    // Color detection thresholds (widened for more reliability)
    // Purple
    private static final int PURPLE_R_MIN = 1000;
    private static final int PURPLE_R_MAX = 1400;
    private static final int PURPLE_G_MIN = 1100;
    private static final int PURPLE_G_MAX = 1500;
    private static final int PURPLE_B_MIN = 1600;
    private static final int PURPLE_B_MAX = 2100;

    // Green
    private static final int GREEN_R_MIN = 350;
    private static final int GREEN_R_MAX = 700;
    private static final int GREEN_G_MIN = 1600;
    private static final int GREEN_G_MAX = 2600;
    private static final int GREEN_B_MIN = 900;
    private static final int GREEN_B_MAX = 1700;

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
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

        // Set motor direction if necessary (e.g., intake.setDirection(DcMotor.Direction.REVERSE);)

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        sorter.setPosition(IN1_POSITION); // Set to default position at start
        
        while (opModeIsActive()) {
            // Set intake power
            intake.setPower(-1.0);

            // Read color sensor values
            int red = colorSensor.red();
            int green = colorSensor.green();
            int blue = colorSensor.blue();
            int alpha = colorSensor.alpha();

            // Sorter State Machine
            switch (sorterState) {
                case IDLE:
                    // If we detect a color, start the sorting sequence.
                    if (isPurple(red, green, blue) || isGreen(red, green, blue)) {
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
            telemetry.addData("Red", red);
            telemetry.addData("Green", green);
            telemetry.addData("Blue", blue);
            telemetry.addData("Alpha", alpha);
            telemetry.addData("Sorter State", sorterState.toString());
            telemetry.addData("Sorter Position", sorter.getPosition());
            telemetry.addData("Intake Power", intake.getPower());
            telemetry.update();

            // Allow other processes to run
            idle();
        }
    }

    private boolean isPurple(int r, int g, int b) {
        return (r >= PURPLE_R_MIN && r <= PURPLE_R_MAX &&
                g >= PURPLE_G_MIN && g <= PURPLE_G_MAX &&
                b >= PURPLE_B_MIN && b <= PURPLE_B_MAX);
    }

    private boolean isGreen(int r, int g, int b) {
        return (r >= GREEN_R_MIN && r <= GREEN_R_MAX &&
                g >= GREEN_G_MIN && g <= GREEN_G_MAX &&
                b >= GREEN_B_MIN && b <= GREEN_B_MAX);
    }
}
