package org.firstinspires.ftc.teamcode.archived.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Turret Encoder Calibration TeleOp
 * 
 * Purpose: Measure encoder ticks per 360° rotation for precise motor calibration
 * 
 * Controls:
 * - Right Stick Y: Apply power to turret motor (+/- 1.0)
 * - Left Stick Y: Adjust power slowly (±0.1 increments) for fine control
 * - A: Reset encoder to 0
 * - B: Mark starting position (snapshots current ticks)
 * - X: Calculate ticks per degree from snapshot
 * - Y: Full rotation test (auto-rotates and measures)
 * - DPad Up: Increase power by 0.05
 * - DPad Down: Decrease power by 0.05
 * 
 * Expected Results:
 * - GoBilda 223 RPM: ~10,240 ticks/360° = ~28.4 ticks/degree
 * - GoBilda 435 RPM: ~8,000 ticks/360° = ~22.2 ticks/degree
 * 
 * Calibration Steps:
 * 1. Press A to reset encoder
 * 2. Use right stick to rotate turret exactly 360° (back to start position)
 * 3. Note the encoder ticks displayed
 * 4. Update TICKS_PER_DEGREE in TurretSubsystem with measured value
 */
@TeleOp(name = "Turret Calibration", group = "Calibration")
public class TurretCalibration extends LinearOpMode {

    private DcMotor turretMotor;

    // Calibration tracking
    private int startingTicks = 0;
    private int currentTicks = 0;
    private double currentPower = 0.0;

    // Snapshots for testing
    private int snapshotTicks = 0;
    private boolean hasSnapshot = false;

    @Override
    public void runOpMode() {
        // === Initialize motor in RUN_WITHOUT_ENCODER mode for manual control ===
        turretMotor = hardwareMap.get(DcMotor.class, "turret");
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretMotor.setPower(0.0);

        telemetry.addLine("=== TURRET ENCODER CALIBRATION ===");
        telemetry.addLine("Goal: Measure ticks per 360° rotation");
        telemetry.addLine("");
        telemetry.addLine("STEPS:");
        telemetry.addLine("1. Press A to reset encoder");
        telemetry.addLine("2. Use Right Stick Y to rotate slowly");
        telemetry.addLine("3. Rotate exactly 360° back to start");
        telemetry.addLine("4. Note total ticks (subtract starting ticks)");
        telemetry.addLine("5. Calculate: ticks_per_degree = total_ticks / 360");
        telemetry.addLine("");
        telemetry.addLine("CONTROLS:");
        telemetry.addLine("Right Stick: Full power control");
        telemetry.addLine("Left Stick: Fine power adjustment (±0.1)");
        telemetry.addLine("DPad U/D: ±0.05 power increments");
        telemetry.addLine("A: Reset encoder | B: Snapshot | X: Calculate");
        telemetry.addLine("Waiting for start...");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // === Get current encoder position ===
            currentTicks = turretMotor.getCurrentPosition();

            // === Handle gamepad input ===
            handleInput();

            // === Apply power to motor ===
            turretMotor.setPower(currentPower);

            // === Display calibration data ===
            displayCalibration();
        }

        // === Cleanup ===
        turretMotor.setPower(0.0);
        telemetry.addLine("Calibration complete");
        telemetry.update();
    }

    /**
     * Handle gamepad input
     */
    private void handleInput() {
        // Right Stick Y: Full power control (1.0 to -1.0)
        if (Math.abs(gamepad1.right_stick_y) > 0.05) {
            currentPower = -gamepad1.right_stick_y;  // Negative because up is positive in stick
        }

        // Left Stick Y: Fine adjustment (smaller increments)
        if (Math.abs(gamepad1.left_stick_y) > 0.05) {
            currentPower += -gamepad1.left_stick_y * 0.01;  // 0.01 per frame = ~0.1 per loop
            currentPower = Math.max(-1.0, Math.min(1.0, currentPower));
        }

        // DPad Up: Increase power by 0.05
        if (gamepad1.dpad_up) {
            currentPower += 0.05;
            currentPower = Math.max(-1.0, Math.min(1.0, currentPower));
            sleep(100);
        }

        // DPad Down: Decrease power by 0.05
        if (gamepad1.dpad_down) {
            currentPower -= 0.05;
            currentPower = Math.max(-1.0, Math.min(1.0, currentPower));
            sleep(100);
        }

        // A: Reset encoder to zero
        if (gamepad1.a) {
            turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            startingTicks = 0;
            currentTicks = 0;
            hasSnapshot = false;
            telemetry.addLine("✓ Encoder reset to 0");
            sleep(200);
        }

        // B: Take snapshot of current ticks
        if (gamepad1.b) {
            snapshotTicks = currentTicks;
            hasSnapshot = true;
            telemetry.addLine("✓ Snapshot taken: " + snapshotTicks + " ticks");
            sleep(200);
        }

        // X: Calculate ticks per degree from snapshot
        if (gamepad1.x) {
            if (hasSnapshot) {
                int deltaTicks = currentTicks - snapshotTicks;
                double ticksPerDegree = (double) Math.abs(deltaTicks) / 360.0;
                telemetry.addLine("=== CALCULATION ===");
                telemetry.addLine("Snapshot: " + snapshotTicks + " ticks");
                telemetry.addLine("Current: " + currentTicks + " ticks");
                telemetry.addLine("Delta: " + deltaTicks + " ticks");
                telemetry.addLine("Ticks per degree: " + String.format("%.2f", ticksPerDegree));
                telemetry.addLine("→ Update TurretSubsystem.TICKS_PER_DEGREE");
            } else {
                telemetry.addLine("No snapshot! Press B first.");
            }
            sleep(200);
        }

        // Y: Auto rotate test (slowly rotate 360° with constant power)
        if (gamepad1.y) {
            autoRotationTest();
        }
    }

    /**
     * Automatic 360° rotation test with measurement
     * Rotates at constant power and measures ticks
     */
    private void autoRotationTest() {
        telemetry.addLine("=== STARTING 360° ROTATION TEST ===");
        telemetry.update();

        // Reset encoder
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sleep(200);

        // Rotate at 0.5 power (moderate speed)
        double testPower = 0.5;
        turretMotor.setPower(testPower);

        int testStartTicks = 0;
        int testEndTicks = 0;

        // Wait for rotation to complete (rough estimate: ~3 seconds at 0.5 power)
        long startTime = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() - startTime < 5000) {
            testEndTicks = turretMotor.getCurrentPosition();
            telemetry.addData("Test Rotation Ticks", testEndTicks);
            telemetry.update();
        }

        turretMotor.setPower(0.0);

        // Calculate results
        int rotationTicks = Math.abs(testEndTicks - testStartTicks);
        double calculatedTicksPerDegree = (double) rotationTicks / 360.0;

        telemetry.addLine("=== AUTO TEST RESULTS ===");
        telemetry.addData("Rotation Ticks", rotationTicks);
        telemetry.addData("Ticks per Degree", String.format("%.2f", calculatedTicksPerDegree));
        telemetry.addLine("→ Update TICKS_PER_DEGREE in TurretSubsystem");
        telemetry.update();

        sleep(3000);  // Show results for 3 seconds
    }

    /**
     * Display calibration telemetry
     */
    private void displayCalibration() {
        telemetry.addLine("=== ENCODER STATUS ===");
        telemetry.addData("Current Ticks", currentTicks);
        telemetry.addData("Current Power", "%.2f", currentPower);
        telemetry.addData("Motor Direction", currentPower > 0 ? "FORWARD" : (currentPower < 0 ? "REVERSE" : "STOPPED"));

        if (hasSnapshot) {
            telemetry.addLine("=== SNAPSHOT ===");
            telemetry.addData("Snapshot Ticks", snapshotTicks);
            telemetry.addData("Delta from Snapshot", currentTicks - snapshotTicks);
            double estimatedDegrees = (currentTicks - snapshotTicks) / 28.4;  // Using default estimate
            telemetry.addData("Estimated Degrees", String.format("%.1f°", estimatedDegrees));
        }

        telemetry.addLine("");
        telemetry.addLine("CURRENT SETTINGS:");
        telemetry.addLine("Press X to calculate ticks per degree");
        telemetry.addLine("Press Y to run auto 360° test");

        telemetry.update();
    }
}
