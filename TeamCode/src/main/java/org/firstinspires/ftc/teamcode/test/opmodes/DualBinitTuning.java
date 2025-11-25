package org.firstinspires.ftc.teamcode.test.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Dual Binit Tuning
 * -----------------
 * Controls two servos: binit (left) and brinit (right)
 *
 * - On init:
 *      binit = 0.0
 *      brinit = 1.0
 * - DPad Up / Down: control binit (+/- 0.05)
 * - DPad Right / Left: control brinit (+/- 0.05)
 * - Gamepad B: binit +0.05, brinit -0.05
 * - Gamepad A: binit -0.05, brinit +0.05
 */
@TeleOp(name = "Dual Binit Tuning", group = "Test")
public class DualBinitTuning extends LinearOpMode {

    private Servo binit;
    private Servo brinit;
    private double binitPos = 0.1;
    private double brinitPos = 0.8;

    @Override
    public void runOpMode() {
        // Initialize servos
        binit = hardwareMap.get(Servo.class, "binit");
        brinit = hardwareMap.get(Servo.class, "brinit");

        // Set initial positions
        binit.setPosition(binitPos);
        brinit.setPosition(brinitPos);

        telemetry.addLine("Dual Binit Tuning Initialized");
        telemetry.addData("binit", binitPos);
        telemetry.addData("brinit", brinitPos);
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // === Individual Control ===
            if (gamepad1.dpad_up) {
                binitPos = Math.min(1.0, binitPos + 0.01);
            }
            if (gamepad1.dpad_down) {
                binitPos = Math.max(0.0, binitPos - 0.01);
            }

            if (gamepad1.dpad_right) {
                brinitPos = Math.min(1.0, brinitPos + 0.01);
            }
            if (gamepad1.dpad_left) {
                brinitPos = Math.max(0.0, brinitPos - 0.01);
            }

            // === Dual Linked Control ===
            if (gamepad1.b) {
                // B: Move binit up, brinit down
                binitPos = Math.min(1.0, binitPos + 0.01);
                brinitPos = Math.max(0.0, brinitPos - 0.01);
            }

            if (gamepad1.a) {
                // A: Inverse of B
                binitPos = Math.max(0.0, binitPos - 0.01);
                brinitPos = Math.min(1.0, brinitPos + 0.01);
            }

            // Apply servo positions
            binit.setPosition(binitPos);
            brinit.setPosition(brinitPos);

            // Telemetry update
            telemetry.addLine("== Dual Binit Tuning ==");
            telemetry.addData("binit", "%.2f", binitPos);
            telemetry.addData("brinit", "%.2f", brinitPos);
            telemetry.addLine("DPad: control individually");
            telemetry.addLine("B/A: move both in opposite directions");
            telemetry.update();

            sleep(100); // Prevent spamming
        }
    }
}
