package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "DkTestBall", group = "Test")
public class DkTestBall extends LinearOpMode {

    private DcMotor intake = null;
    private Servo sorter = null;
    private ColorSensor colorSensor = null;

    // Servo positions
    private static final double IN1_POSITION = 0.035;
    private static final double IN2_POSITION = 0.441;
    private static final double IN3_POSITION = 0.782;

    // Color detection thresholds (based on provided values)
    // Purple
    private static final int PURPLE_R_MIN = 1100;
    private static final int PURPLE_R_MAX = 1300;
    private static final int PURPLE_G_MIN = 1200;
    private static final int PURPLE_G_MAX = 1400;
    private static final int PURPLE_B_MIN = 1700;
    private static final int PURPLE_B_MAX = 2000;

    // Green
    private static final int GREEN_R_MIN = 400;
    private static final int GREEN_R_MAX = 600;
    private static final int GREEN_G_MIN = 1700;
    private static final int GREEN_G_MAX = 2500;
    private static final int GREEN_B_MIN = 1000;
    private static final int GREEN_B_MAX = 1600;

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

        while (opModeIsActive()) {
            // Set intake power
            intake.setPower(-1.0);

            // Read color sensor values
            int red = colorSensor.red();
            int green = colorSensor.green();
            int blue = colorSensor.blue();
            int alpha = colorSensor.alpha();

            // Check for purple
            if (isPurple(red, green, blue)) {
                // Cycle through positions for purple
                sorter.setPosition(IN1_POSITION);
                sleep(500);
                sorter.setPosition(IN2_POSITION);
                sleep(500);
                sorter.setPosition(IN3_POSITION);
                sleep(500);
            }
            // Check for green
            else if (isGreen(red, green, blue)) {
                 // Cycle through positions for green
                sorter.setPosition(IN1_POSITION);
                sleep(500);
                sorter.setPosition(IN2_POSITION);
                sleep(500);
                sorter.setPosition(IN3_POSITION);
                sleep(500);
            } else {
                // Default position or do nothing
                 sorter.setPosition(IN1_POSITION);
            }

            // Telemetry for debugging
            telemetry.addData("Red", red);
            telemetry.addData("Green", green);
            telemetry.addData("Blue", blue);
            telemetry.addData("Alpha", alpha);
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
