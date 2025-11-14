package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.util.GlobalData;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Simple OpMode to control the sorter and binit servos.
 * - DPad Up: Increase sorter position by 0.1
 * - DPad Down: Decrease sorter position by 0.1
 * - DPad Left: Decrease binit position by 0.1
 * - DPad Right: Increase binit position by 0.1
 */
@TeleOp(name = "Sorter and Binit Tuning", group = "Test")
public class SorterANDFeed_Tuning extends LinearOpMode {

    private Servo sorter;  // Declare the sorter servo
    private Servo binit;   // Declare the binit servo
    private double sorterPosition = 0.5;  // Start with sorter position at 0.5 (default mid position)
    private double binitPosition = 0.35;  // Start with binit position at 0.35 (default)

    @Override
    public void runOpMode() {
        // Initialize the servos
        sorter = hardwareMap.get(Servo.class, "sorter");
        binit = hardwareMap.get(Servo.class, "binit");

        // Set initial positions for both servos
        sorter.setPosition(sorterPosition);
        binit.setPosition(binitPosition);

        telemetry.addData("Sorter Position", sorterPosition);
        telemetry.addData("Binit Position", binitPosition);
        telemetry.update();

        // Wait for the start button to be pressed
        waitForStart();

        // Loop while the OpMode is running
        while (opModeIsActive()) {
            // Check DPad inputs to adjust the sorter position
            if (gamepad1.dpad_up) {
                // Increase the sorter position by 0.1, making sure it stays between 0.0 and 1.0
                sorterPosition = Math.min(1.0, sorterPosition + 0.001);
            }
            if (gamepad1.dpad_down) {
                // Decrease the sorter position by 0.1, making sure it stays between 0.0 and 1.0
                sorterPosition = Math.max(0.0, sorterPosition - 0.001);
            }

            // Check DPad inputs to adjust the binit position
            if (gamepad1.dpad_left) {
                // Decrease the binit position by 0.1, making sure it stays between 0.0 and 1.0
                binitPosition = Math.max(0.0, binitPosition - 0.001);
            }
            if (gamepad1.dpad_right) {
                // Increase the binit position by 0.1, making sure it stays between 0.0 and 1.0
                binitPosition = Math.min(1.0, binitPosition + 0.001);
            }

            // Set the servo positions to the current values
            sorter.setPosition(sorterPosition);
            binit.setPosition(binitPosition);

            // Update the telemetry with the current servo positions
            telemetry.addData("Sorter Position", sorterPosition);
            telemetry.addData("Binit Position", binitPosition);
            telemetry.update();

            // Optional small sleep to prevent unnecessary updates (can be adjusted)
            sleep(20);
        }
    }
}
