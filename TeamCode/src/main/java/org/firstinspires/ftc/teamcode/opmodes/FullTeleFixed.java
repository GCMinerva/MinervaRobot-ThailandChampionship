package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * FullTeleFixed - Enhanced TeleOp with PedroPathing Integration (OpMode version)
 *
 * Gamepad2 (Driver):
 * - Left/Right Stick: Manual drivetrain control (ALWAYS uses direct motor control)
 *   - During auto-nav: Override with manual control, release to resume path
 *   - Normal mode: Direct mecanum motor control
 * - A Button: Auto-navigate to position (71.7, 67.1, 132.2°) [can press multiple times]
 *   - Auto-nav completes when within 5% position tolerance (~4.9 inches) and 5° heading
 *   - Auto-cancels after 5 seconds if not completed
 *   - Full manual control available during navigation
 * - B Button: Cancel auto-navigation (stops path following, restores manual control)
 * - D-Pad Up/Down: Adjust outtake speed (+/- 0.05, 250ms delay)
 * - X Button: Start outtake motor
 * - Y Button: Stop outtake motor
 *
 * Gamepad1 (Operator):
 * - Left Bumper: sorter servo shoot (0.55 → 1.0 → 0.5s delay → 0.55)
 * - Right Bumper: binit (Axon) servo sequence (1.0 → 0.5s delay → 0.65)
 * - A Button: Cycle sorter input mode (0 → 0.375 → 0.75 → loop)
 * - B Button: Cycle sorter shoot mode (0.55 → 0.9 → 0.15 → loop)
 * - X Button: Start intake motor (-0.75 power)
 * - Y Button: Stop intake motor
 */
@TeleOp(name="FullTeleFixed")
public class FullTeleFixed extends OpMode {

    // PedroPathing
    private Follower follower;

    // Drive motors (for manual control fallback)
    private DcMotor frontLeft, frontRight, backLeft, backRight;

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
    private boolean gp2_bPressed = false;
    private boolean gp1_xPressed = false;
    private boolean gp1_yPressed = false;
    private boolean gp1_aPressed = false;
    private boolean gp1_bPressed = false;
    private boolean gp1_rbPressed = false;
    private boolean gp1_lbPressed = false;

    // Sorter cycling positions
    private int sorterShootMode = 0;  // For gamepad1.b: 0=0.55, 1=0.9, 2=0.15
    private int sorterInputMode = 0;  // For gamepad1.a: 0=0, 1=0.375, 2=0.75

    // Auto-navigation
    private boolean autoNavigating = false;
    private boolean manualOverride = false;
    private PathChain autoPath;
    private long autoNavStartTime = 0;
    private final long AUTO_NAV_TIMEOUT_MS = 5000;  // 5 seconds

    // Starting pose (90 degrees to match field orientation)
    private final Pose startPose = new Pose(58, 0, Math.toRadians(90));
    private final Pose targetPose = new Pose(71.7, 67.1, Math.toRadians(132.2));

    // PIDF error tolerance: 5% of target position magnitude
    private final double POSITION_TOLERANCE_PERCENT = 0.05;
    private final double positionTolerance = Math.hypot(targetPose.getX(), targetPose.getY()) * POSITION_TOLERANCE_PERCENT;
    private final double HEADING_TOLERANCE_DEGREES = 5.0;

    @Override
    public void init() {
        // Initialize PedroPathing follower
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        // CRITICAL: Update follower multiple times to fully initialize all internal pose objects
        // The follower has multiple internal pose systems that need initialization
        for (int i = 0; i < 5; i++) {
            follower.update();
        }

        // Initialize drive motors (for fallback manual control)
        frontLeft = hardwareMap.get(DcMotor.class, "fl");
        frontRight = hardwareMap.get(DcMotor.class, "fr");
        backLeft = hardwareMap.get(DcMotor.class, "bl");
        backRight = hardwareMap.get(DcMotor.class, "br");

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
    }

    @Override
    public void init_loop() {
        // Keep updating follower until pose is initialized
        follower.update();
        Pose pose = follower.getPose();

        if (pose != null) {
            telemetry.addData("Status", "Ready! Pose initialized");
            // Swap X and Y to match field orientation, and reverse Y
            telemetry.addData("Position", "X: %.1f, Y: %.1f", pose.getY(), -pose.getX());
        } else {
            telemetry.addData("Status", "WAITING for Pinpoint...");
            telemetry.addData("Info", "Check Pinpoint hardware connection");
        }
        telemetry.update();
    }

    @Override
    public void loop() {
        // Update follower FIRST
        follower.update();

        // Get current pose
        Pose currentPose = follower.getPose();

        // ===== GAMEPAD2: DRIVETRAIN CONTROL =====

        // Get joystick inputs
        double forwardSpeed = -gamepad2.left_stick_y;
        double strafeSpeed = gamepad2.left_stick_x;
        double turnSpeed = gamepad2.right_stick_x;

        // Check if joystick is being used (with small deadzone)
        boolean joystickActive = Math.abs(forwardSpeed) > 0.1 ||
                                 Math.abs(strafeSpeed) > 0.1 ||
                                 Math.abs(turnSpeed) > 0.1;

        // Check for 5-second timeout on auto-navigation
        if (autoNavigating && (System.currentTimeMillis() - autoNavStartTime) > AUTO_NAV_TIMEOUT_MS) {
            cancelAutoNavigation();
            telemetry.addData("Auto Nav", "CANCELLED - 5 second timeout");
        }

        // Manual override: Use joystick during auto-navigation without canceling
        if (autoNavigating && joystickActive && !manualOverride) {
            // Joystick just became active - break path following to enable manual control
            manualOverride = true;
            follower.breakFollowing();
            telemetry.addData("Override", "Breaking path for manual control");
        } else if (autoNavigating && !joystickActive && manualOverride) {
            // Joystick just released - resume path following
            manualOverride = false;
            if (currentPose != null) {
                startAutoNavigation(currentPose);
                telemetry.addData("Override", "Resuming auto-navigation");
            }
        }

        // Control logic: ALWAYS use direct motor control when not auto-navigating
        // This completely bypasses the follower to avoid any state interference
        if (!autoNavigating || manualOverride) {
            // MANUAL MODE: Use direct motor control
            setManualDrive(forwardSpeed, strafeSpeed, turnSpeed);

            if (manualOverride) {
                telemetry.addData("Drive Mode", "MANUAL OVERRIDE");
            } else {
                telemetry.addData("Drive Mode", "MANUAL");
            }
        }
        // If autoNavigating && !manualOverride: follower continues path following automatically

        // Gamepad2 A: Auto-navigate to target position (can be pressed multiple times)
        if (gamepad2.a && !gp2_aPressed && currentPose != null) {
            gp2_aPressed = true;
            startAutoNavigation(currentPose);
        }
        if (!gamepad2.a) gp2_aPressed = false;

        // Gamepad2 B: Cancel auto-navigation
        if (gamepad2.b && !gp2_bPressed) {
            gp2_bPressed = true;
            if (autoNavigating) {
                cancelAutoNavigation();
                telemetry.addData("Auto Nav", "CANCELLED by user (B button)");
            }
        }
        if (!gamepad2.b) gp2_bPressed = false;

        // Check auto-navigation completion with 5% position tolerance
        if (autoNavigating && !follower.isBusy() && !manualOverride) {
            // Check if we're within 5% tolerance of target position
            double distanceToTarget = Math.hypot(
                currentPose.getX() - targetPose.getX(),
                currentPose.getY() - targetPose.getY()
            );
            double headingError = Math.abs(Math.toDegrees(currentPose.getHeading() - targetPose.getHeading()));

            if (distanceToTarget <= positionTolerance && headingError <= HEADING_TOLERANCE_DEGREES) {
                cancelAutoNavigation();  // Use the cancel method to properly clean up
                telemetry.addData("Auto Nav", "COMPLETED (within %.2f\" tolerance)", positionTolerance);
            }
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
            new Thread(() -> {
                try {
                    Thread.sleep(500);
                    sorter.setPosition(0.55);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
            }).start();
        }
        if (!gamepad1.left_bumper) gp1_lbPressed = false;

        // Gamepad1 Right Bumper: binit (Axon) sequence (1.0 -> 500ms -> 0.65)
        if (gamepad1.right_bumper && !gp1_rbPressed) {
            gp1_rbPressed = true;
            binit.setPosition(1.0);
            new Thread(() -> {
                try {
                    Thread.sleep(500);
                    binit.setPosition(0.35);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
            }).start();
        }
        if (!gamepad1.right_bumper) gp1_rbPressed = false;

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
        if (currentPose != null) {
            // Swap X and Y to match field orientation, and reverse Y
            telemetry.addData("Position", "X: %.1f, Y: %.1f", currentPose.getX(), currentPose.getY());
            telemetry.addData("Heading", "%.1f°", Math.toDegrees(currentPose.getHeading()));

            // Show distance to target when auto-navigating
            if (autoNavigating) {
                double distanceToTarget = Math.hypot(
                    currentPose.getX() - targetPose.getX(),
                    currentPose.getY() - targetPose.getY()
                );
                telemetry.addData("Distance to Target", "%.2f\" (tolerance: %.2f\")",
                    distanceToTarget, positionTolerance);
            }
        } else {
            telemetry.addData("Position", "INITIALIZING...");
        }

        if (autoNavigating) {
            long elapsedTime = System.currentTimeMillis() - autoNavStartTime;
            long remainingTime = (AUTO_NAV_TIMEOUT_MS - elapsedTime) / 1000;
            String autoNavStatus = manualOverride ? "ACTIVE (Manual Override)" : "ACTIVE (Following Path)";
            telemetry.addData("Auto Nav", "%s [%ds left]", autoNavStatus, remainingTime);
        } else {
            telemetry.addData("Auto Nav", "Idle");
        }
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

    /**
     * Start autonomous navigation to target position
     */
    private void startAutoNavigation(Pose currentPose) {
        autoPath = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(currentPose.getX(), currentPose.getY(), currentPose.getHeading()),
                        targetPose
                ))
                .setLinearHeadingInterpolation(currentPose.getHeading(), targetPose.getHeading())
                .build();

        follower.followPath(autoPath, /* holdEnd = */ false);  // Changed to false to prevent holding position
        autoNavigating = true;
        autoNavStartTime = System.currentTimeMillis();  // Start the 5-second timer

        telemetry.addData("Auto Nav", "Started to (%.1f, %.1f, %.1f°)",
                targetPose.getX(), targetPose.getY(), Math.toDegrees(targetPose.getHeading()));
        telemetry.update();
    }

    /**
     * Cancel auto-navigation and restore manual control
     */
    private void cancelAutoNavigation() {
        // Break path following
        follower.breakFollowing();

        // Reset all auto-nav state
        autoNavigating = false;
        manualOverride = false;

        // Directly stop all motors to immediately release control
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    /**
     * Manual mecanum drive control (fallback if PedroPathing fails)
     */
    private void setManualDrive(double forward, double strafe, double turn) {
        // Mecanum drive calculations
        double frontLeftPower = forward + strafe + turn;
        double frontRightPower = forward - strafe - turn;
        double backLeftPower = forward - strafe + turn;
        double backRightPower = forward + strafe - turn;

        // Normalize powers to ensure they don't exceed 1.0
        double maxPower = Math.max(Math.abs(frontLeftPower), Math.max(Math.abs(frontRightPower),
                Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))));

        if (maxPower > 1.0) {
            frontLeftPower /= maxPower;
            frontRightPower /= maxPower;
            backLeftPower /= maxPower;
            backRightPower /= maxPower;
        }

        // Set motor powers
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
    }

    @Override
    public void stop() {
        outtake.setPower(0);
        intake.setPower(0);
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
}
