package org.firstinspires.ftc.teamcode.archived.opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * FullTele - Enhanced TeleOp with PedroPathing Integration
 *
 * Gamepad2 (Driver):
 * - Left/Right Stick: Manual drivetrain control
 * - A Button: Auto-navigate to position (71.8, -67.2, 132.6°)
 * - D-Pad Up/Down: Adjust outtake speed (+/- 0.05, 250ms delay)
 * - X Button: Start outtake motor
 * - Y Button: Stop outtake motor
 *
 * Gamepad1 (Operator):
 * - Left Bumper: sorter servo shoot (0.55 → 1.0 → 0.5s delay → 0.55)
 * - Right Bumper: binit (Axon) servo sequence (1.0 → 0.5s delay → 0.35)
 * - A Button: Cycle sorter input mode (0 → 0.375 → 0.75 → loop)
 * - B Button: Cycle sorter shoot mode (0.55 → 0.9 → 0.15 → loop)
 * - X Button: Start intake motor (-0.75 power)
 * - Y Button: Stop intake motor
 */
@TeleOp(name="FullTele")
public class FullTele extends LinearOpMode {

    // PedroPathing
    private Follower follower;

    // Motors
    private DcMotor outtake, intake;

    // Servos (only 2 physical servos)
    private Servo binit;  // Hardware: "binit" (also called Axon)
    private Servo sorter; // Hardware: "sorter"

    // Control variables
    private double outtakePower = 0;
    private double outtakeSpeed = 0.5;
    private long lastSpeedChangeTime = 0;
    private double intakePower = 0;

    // Button debouncing
    private boolean gp2_xPressed = false;
    private boolean gp2_yPressed = false;
    private boolean gp2_aPressed = false;
    private boolean gp1_xPressed = false;
    private boolean gp1_yPressed = false;
    private boolean gp1_aPressed = false;
    private boolean gp1_bPressed = false;
    private boolean gp1_lbPressed = false;

    // Sorter cycling positions
    private int sorterShootMode = 0;  // For gamepad1.b: 0=0.55, 1=0.9, 2=0.15
    private int sorterInputMode = 0;  // For gamepad1.a: 0=0, 1=0.375, 2=0.75

    // Auto-navigation
    private boolean autoNavigating = false;
    private PathChain autoPath;

    // Starting pose (90 degrees to match field orientation)
    // Note: Pose(X, Y) but our field uses swapped coords, so Pose(fieldY, fieldX)
    private final Pose startPose = new Pose(0, 58, Math.toRadians(90));
    private final Pose targetPose = new Pose(-67.2, 71.8, Math.toRadians(132.6));

    // Cached pose to avoid null pointer
    private Pose cachedPose;

    @Override
    public void runOpMode() {
        // Initialize PedroPathing follower
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        // CRITICAL: Update follower multiple times to fully initialize all internal pose objects
        for (int i = 0; i < 5; i++) {
            follower.update();
        }

        // Initialize cached pose
        cachedPose = startPose;

        // Verify pose objects are not null
        if (startPose == null || targetPose == null) {
            telemetry.addData("ERROR", "Pose objects not initialized!");
            telemetry.update();
            throw new RuntimeException("startPose or targetPose is null");
        }

        // Initialize motors
        outtake = hardwareMap.get(DcMotor.class, "outtake");
        intake = hardwareMap.get(DcMotor.class, "intake");

        outtake.setDirection(DcMotor.Direction.FORWARD);
        outtake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Map servos (only 2 physical servos)
        binit = hardwareMap.get(Servo.class, "binit");    // binit servo (also called Axon)
        sorter = hardwareMap.get(Servo.class, "sorter");  // sorter servo

        binit.setPosition(0.35);
        sorter.setPosition(0.55);  // Start in shoot position

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Starting Pose", "X: %.1f, Y: %.1f, H: 90°", startPose.getX(), startPose.getY());
        telemetry.update();

        waitForStart();

        // Initial follower update to initialize pose
        follower.update();

        // Update cached pose from follower after first update
        Pose initialPose = follower.getPose();
        if (initialPose != null) {
            cachedPose = initialPose;
        }

        while (opModeIsActive()) {
            // Update follower FIRST (must be called before any drive commands)
            follower.update();

            // Update cached pose (with safety fallback to startPose)
            Pose currentPose = follower.getPose();
            if (currentPose != null) {
                cachedPose = currentPose;
            } else if (cachedPose == null) {
                // Fallback: if both are null, use startPose
                cachedPose = startPose;
            }

            // ===== GAMEPAD2: DRIVETRAIN CONTROL =====

            // Only use TeleOp drive if follower pose is initialized
            if (currentPose != null) {
                if (!autoNavigating) {
                    // Manual control
                    double forwardSpeed = -gamepad2.left_stick_y;
                    double strafeSpeed = gamepad2.left_stick_x;
                    double turnSpeed = gamepad2.right_stick_x;  // Fixed: removed negative to correct rotation direction

                    follower.setTeleOpDrive(forwardSpeed, strafeSpeed, turnSpeed, true);
                }
            } else {
                // Pose not ready yet - show warning in telemetry
                telemetry.addData("WARNING", "Waiting for Pinpoint initialization...");
            }

            // Gamepad2 A: Auto-navigate to target position
            if (gamepad2.a && !gp2_aPressed) {
                gp2_aPressed = true;
                startAutoNavigation();
            }
            if (!gamepad2.a) gp2_aPressed = false;

            // Check auto-navigation completion
            if (autoNavigating && !follower.isBusy()) {
                autoNavigating = false;
                telemetry.addData("Auto Nav", "Completed");
            }

            // ===== GAMEPAD2: OUTTAKE CONTROL =====

            // Gamepad2 X: Start outtake motor
            if (gamepad2.x && !gp2_xPressed) {
                gp2_xPressed = true;
                outtakePower = outtakeSpeed;
            }
            if (!gamepad2.x) gp2_xPressed = false;

            // Gamepad2 Y: Stop outtake motor
            if (gamepad2.y && !gp2_yPressed) {
                gp2_yPressed = true;
                outtakePower = 0;
            }
            if (!gamepad2.y) gp2_yPressed = false;

            // Gamepad2 D-Pad: Control outtake speed (250ms delay)
            long currentTime = System.currentTimeMillis();
            if (currentTime - lastSpeedChangeTime >= 250) {
                if (gamepad2.dpad_up) {
                    outtakeSpeed = Math.min(1.0, outtakeSpeed + 0.05);
                    lastSpeedChangeTime = currentTime;
                }
                if (gamepad2.dpad_down) {
                    outtakeSpeed = Math.max(0.0, outtakeSpeed - 0.05);
                    lastSpeedChangeTime = currentTime;
                }
            }

            // Real-time outtake speed update
            if (outtakePower > 0) {
                outtakePower = outtakeSpeed;
            }

            outtake.setPower(outtakePower);

            // ===== GAMEPAD1: SERVO CONTROL =====

            // Gamepad1 Left Bumper: sorter servo sequence (0.55 -> 1.0 -> wait 500ms -> 0.55)
            if (gamepad1.left_bumper && !gp1_lbPressed) {
                gp1_lbPressed = true;
                sorter.setPosition(1.0);
                sleep(500);
                sorter.setPosition(0.55);
            }
            if (!gamepad1.left_bumper) gp1_lbPressed = false;

            // Gamepad1 Right Bumper: binit (Axon) sequence (1.0 -> 500ms -> 0.35)
            if (gamepad1.right_bumper) {
                binit.setPosition(1.0);
                sleep(500);
                binit.setPosition(0.35);
            }

            // ===== GAMEPAD1: SORTER SERVO CONTROL =====

            // Gamepad1 A: Cycle sorter input positions (0 -> 0.375 -> 0.75 -> 0)
            if (gamepad1.a && !gp1_aPressed) {
                gp1_aPressed = true;
                sorterInputMode = (sorterInputMode + 1) % 3;

                switch (sorterInputMode) {
                    case 0:
                        sorter.setPosition(0.0);
                        break;
                    case 1:
                        sorter.setPosition(0.375);
                        break;
                    case 2:
                        sorter.setPosition(0.75);
                        break;
                }
            }
            if (!gamepad1.a) gp1_aPressed = false;

            // Gamepad1 B: Cycle sorter shoot positions (0.55 -> 0.9 -> 0.15 -> 0.55)
            if (gamepad1.b && !gp1_bPressed) {
                gp1_bPressed = true;
                sorterShootMode = (sorterShootMode + 1) % 3;

                switch (sorterShootMode) {
                    case 0:
                        sorter.setPosition(0.55);
                        break;
                    case 1:
                        sorter.setPosition(0.9);
                        break;
                    case 2:
                        sorter.setPosition(0.15);
                        break;
                }
            }
            if (!gamepad1.b) gp1_bPressed = false;

            // ===== GAMEPAD1: INTAKE CONTROL =====

            // Gamepad1 X: Start intake motor
            if (gamepad1.x && !gp1_xPressed) {
                gp1_xPressed = true;
                intakePower = -0.75;
            }
            if (!gamepad1.x) gp1_xPressed = false;

            // Gamepad1 Y: Stop intake motor
            if (gamepad1.y && !gp1_yPressed) {
                gp1_yPressed = true;
                intakePower = 0;
            }
            if (!gamepad1.y) gp1_yPressed = false;

            intake.setPower(intakePower);

            // ===== TELEMETRY =====

            telemetry.addData("=== DRIVETRAIN ===", "");
            if (cachedPose != null) {
                // Swap X and Y to match field orientation, and reverse Y
                telemetry.addData("Position", "X: %.1f, Y: %.1f", cachedPose.getY(), -cachedPose.getX());
                telemetry.addData("Heading", "%.1f°", Math.toDegrees(cachedPose.getHeading()));
            } else {
                telemetry.addData("Position", "INITIALIZING...");
            }
            telemetry.addData("Auto Nav", autoNavigating ? "ACTIVE" : "Idle");
            telemetry.addData("", "");

            telemetry.addData("=== OUTTAKE (GP2) ===", "");
            telemetry.addData("Power", "%.2f", outtakePower);
            telemetry.addData("Speed", "%.2f", outtakeSpeed);
            telemetry.addData("Status", outtakePower > 0 ? "RUNNING" : "STOPPED");
            telemetry.addData("", "");

            telemetry.addData("=== INTAKE (GP1) ===", "");
            telemetry.addData("Power", "%.2f", intakePower);
            telemetry.addData("Status", intakePower != 0 ? "RUNNING" : "STOPPED");
            telemetry.addData("", "");

            telemetry.addData("=== SERVOS ===", "");
            telemetry.addData("binit (Axon)", "%.2f", binit.getPosition());
            telemetry.addData("sorter Position", "%.2f", sorter.getPosition());
            telemetry.addData("sorter Input Mode", "%d (0/0.375/0.75)", sorterInputMode);
            telemetry.addData("sorter Shoot Mode", "%d (0.55/0.9/0.15)", sorterShootMode);

            telemetry.update();
        }
    }

    /**
     * Start autonomous navigation to target position
     */
    private void startAutoNavigation() {
        // Safety check: ensure cachedPose is not null
        if (cachedPose == null) {
            telemetry.addData("Auto Nav", "ERROR: Pose not initialized");
            telemetry.update();
            return;
        }

        // Use cached pose to create path
        autoPath = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(cachedPose.getX(), cachedPose.getY(), cachedPose.getHeading()),
                        targetPose
                ))
                .build();

        follower.followPath(autoPath);
        autoNavigating = true;

        telemetry.addData("Auto Nav", "Started to (%.1f, %.1f, %.1f°)",
                targetPose.getX(), targetPose.getY(), Math.toDegrees(targetPose.getHeading()));
        telemetry.update();
    }
}
