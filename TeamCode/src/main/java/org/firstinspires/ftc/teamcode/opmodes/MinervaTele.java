package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.Range;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.qualcomm.robotcore.hardware.ColorSensor;
@Disabled
@TeleOp(name="MinervaTele Op")
public class MinervaTele extends OpMode {
    boolean lbPressed = false;
    boolean shooterOn = false;
    boolean yPressed = false;
    boolean rbPressed = false;
    boolean rtPressed = false;

    private Follower follower;
    private boolean sorterBusy = false;

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private ColorSensor colorSensor;
    private DcMotor outtake1, outtake2, intake, turret;
    private Servo binit, sorter;
    private double outtakePower = 0;
    private double outtakeSpeed = 0.5;
    private double intakePower = 0;

    private boolean gp2_xPressed = false;
    private boolean gp2_bPressed = false;
    // Gamepad 1 toggles
    private boolean gp1_xPressed = false;
    private boolean gp1_yPressed = false;
    private boolean gp1_aPressed = false;
    private boolean gp1_bPressed = false;
    private boolean gp1_rbPressed = false;

    private boolean autoNavigating = false;
    private PathChain autoPath;
    private long autoNavStartTime = 0;
    private final long AUTO_NAV_TIMEOUT_MS = 5000;

    // Drive speed multiplier
    private double driveSpeedMultiplier = 1.0;

    // Poses
    private final Pose startPose = new Pose(58, 0, Math.toRadians(90));
    
    // Targets
    private final Pose POSE_X = new Pose(14, 135, Math.toRadians(110));
    private final Pose POSE_Y = new Pose(71.7, 67.1, Math.toRadians(132.2));
    private final Pose POSE_A = new Pose(39, 34, Math.toRadians(90));

    // We track the current target just for tolerance checking, or we can just use a generic check
    private Pose currentTargetPose = null;

    private final double POSITION_TOLERANCE_PERCENT = 0.05;
    // Since targets vary, we might just use a fixed distance tolerance or dynamic
    private final double FIXED_POSITION_TOLERANCE = 1.0; // 1 inch tolerance
    private final double HEADING_TOLERANCE_DEGREES = 5.0;

    // ===== Sorter system (3 slots) =====
    // 0 = Empty, 1 = Purple, 2 = Green
    int[] slots = {0, 0, 0};
    int currentSlot = 0;
    double[] loadPositions = {0.15, 0.533, 0.918};
    double[] shootPositions = {0.751, 0, 0.354};

    // loading state (non-blocking)
    private boolean loading = false;
    private long loadStartTime = 0;
    private int loadingSlot = -1;
    private final int LOAD_DELAY_MS = 700;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        for (int i = 0; i < 5; i++) {
            follower.update();
        }

        frontLeft = hardwareMap.get(DcMotor.class, "fl");
        frontRight = hardwareMap.get(DcMotor.class, "fr");
        backLeft = hardwareMap.get(DcMotor.class, "bl");
        backRight = hardwareMap.get(DcMotor.class, "br");

        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

        outtake1 = hardwareMap.get(DcMotor.class, "outtake1");
        outtake2 = hardwareMap.get(DcMotor.class, "outtake2");
        intake = hardwareMap.get(DcMotor.class, "intake");
        turret = hardwareMap.get(DcMotor.class, "turret");

        outtake1.setDirection(DcMotor.Direction.FORWARD);
        outtake2.setDirection(DcMotor.Direction.REVERSE);
        outtake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        binit = hardwareMap.get(Servo.class, "binit");
        sorter = hardwareMap.get(Servo.class, "sorter");

        binit.setPosition(0.95);
        sorter.setPosition(0.15);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        follower.update();
        Pose pose = follower.getPose();

        if (pose != null) {
            telemetry.addData("Status", "Ready");
            telemetry.addData("Position", "X: %.1f, Y: %.1f", pose.getY(), -pose.getX());
        } else {
            telemetry.addData("Status", "WAITING for Pinpoint");
        }
        telemetry.update();
    }

    @Override
    public void loop() {
        int r = colorSensor.red();
        int g = colorSensor.green();
        int b = colorSensor.blue();

        follower.update();
        Pose currentPose = follower.getPose();

        double forwardSpeed = -gamepad1.left_stick_y;
        double strafeSpeed = gamepad1.left_stick_x;
        double turnSpeed = gamepad1.right_stick_x;

        boolean joystickActive = Math.abs(forwardSpeed) > 0.1 || Math.abs(strafeSpeed) > 0.1 || Math.abs(turnSpeed) > 0.1;

        // --- Auto Navigation Timeout ---
        if (autoNavigating && (System.currentTimeMillis() - autoNavStartTime) > AUTO_NAV_TIMEOUT_MS) {
            cancelAutoNavigation();
            telemetry.addData("Auto Nav", "CANCELLED - timeout");
        }

        // --- Manual Override (Joystick) ---
        if (autoNavigating && joystickActive) {
            cancelAutoNavigation();
        }

        // --- Manual Override (Right Bumper) ---
        if (gamepad1.right_bumper && !gp1_rbPressed) {
            gp1_rbPressed = true;
            cancelAutoNavigation();
        }
        if (!gamepad1.right_bumper) gp1_rbPressed = false;


        // --- Speed Toggle (Button B) ---
        if (gamepad1.b && !gp1_bPressed) {
            gp1_bPressed = true;
            if (driveSpeedMultiplier == 1.0) {
                driveSpeedMultiplier = 0.5;
            } else {
                driveSpeedMultiplier = 1.0;
            }
        }
        if (!gamepad1.b) gp1_bPressed = false;


        // --- Manual Drive Control ---
        if (!autoNavigating) {
            setManualDrive(forwardSpeed, strafeSpeed, turnSpeed);
        }

        // --- Auto Nav Triggers ---
        // Button A -> Pose A
        if (gamepad1.a && !gp1_aPressed && currentPose != null) {
            gp1_aPressed = true;
            startAutoNavigation(currentPose, POSE_A);
        }
        if (!gamepad1.a) gp1_aPressed = false;

        // Button X -> Pose X
        if (gamepad1.x && !gp1_xPressed && currentPose != null) {
            gp1_xPressed = true;
            startAutoNavigation(currentPose, POSE_X);
        }
        if (!gamepad1.x) gp1_xPressed = false;

        // Button Y -> Pose Y
        if (gamepad1.y && !gp1_yPressed && currentPose != null) {
            gp1_yPressed = true;
            startAutoNavigation(currentPose, POSE_Y);
        }
        if (!gamepad1.y) gp1_yPressed = false;


        // --- Auto Completion Check ---
        if (autoNavigating && !follower.isBusy() && currentTargetPose != null) {
            double distanceToTarget = Math.hypot(
                    currentPose.getX() - currentTargetPose.getX(),
                    currentPose.getY() - currentTargetPose.getY()
            );
            double headingError = Math.abs(Math.toDegrees(currentPose.getHeading() - currentTargetPose.getHeading()));

            if (distanceToTarget <= FIXED_POSITION_TOLERANCE && headingError <= HEADING_TOLERANCE_DEGREES) {
                cancelAutoNavigation();
            }
        }

        if (gamepad2.left_bumper && !lbPressed) {
            lbPressed = true;
            shooterOn = !shooterOn;

            if (shooterOn) {
                outtakePower = outtakeSpeed;   // Turn on shooter
            } else {
                outtakePower = 0;              // Turn off shooter
            }
        }
        if (!gamepad2.left_bumper) lbPressed = false;

        if (gamepad2.right_bumper && !rbPressed) {
            rbPressed = true;
            outtakeSpeed = Math.min(1.0, outtakeSpeed + 0.05);
        }
        if (!gamepad2.right_bumper) rbPressed = false;

        if (gamepad2.right_trigger > 0.1 && !rtPressed) {
            rtPressed = true;
            outtakeSpeed = Math.max(0.0, outtakeSpeed - 0.05);
        }
        if(gamepad2.right_trigger < 0.1) rtPressed = false;

        if (outtakePower > 0) {
            outtakePower = outtakeSpeed;
        }

        outtake1.setPower(outtakePower);
        outtake2.setPower(outtakePower);

        // Turret control with right joystick (gamepad2)
        double turretPower = Range.clip(gamepad2.right_stick_y, -1, 1);
        turret.setPower(turretPower);

        // binit position
        if (gamepad2.b && !gp2_bPressed && Math.abs(sorter.getPosition() - 0.15) > 0.05 && Math.abs(sorter.getPosition() - 0.533) > 0.05 && Math.abs(sorter.getPosition() - 0.918) > 0.05) {
            gp2_bPressed = true;
            binit.setPosition(0.13);
            new Thread(() -> {
                try {
                    Thread.sleep(500);
                    binit.setPosition(0.95);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
            }).start();
        }
        if (!gamepad2.b) gp2_bPressed = false;

        // === LOADING (non-blocking) ===
        if (gamepad2.y && !yPressed && !loading) {
            yPressed = true;
            int next = findNextSlot(0); // Find empty slot
            if (next != -1) {
                currentSlot = next;
                sorter.setPosition(loadPositions[currentSlot]);

                // Start loading state
                loading = true;
                loadStartTime = System.currentTimeMillis();
                loadingSlot = currentSlot;
            }
        }
        if (!gamepad2.y) yPressed = false;

        // Read color after delay
        if (loading && (System.currentTimeMillis() - loadStartTime >= LOAD_DELAY_MS)) {
            if (g > r && g > b && g > 3000) {
                slots[loadingSlot] = 2; // Green
            } else if (r > 500 && b > 500 && g > 500) {
                slots[loadingSlot] = 1; // Purple
            } else {
                slots[loadingSlot] = 0; // Unknown
            }

            telemetry.addData("LoadSlot", loadingSlot);
            telemetry.addData("LoadColor(RGB)", "%d %d %d", r, g, b);

            // reset loading state
            loading = false;
            loadingSlot = -1;
        }

        if (gamepad2.x && !gp2_xPressed) { // Shoot
            gp2_xPressed = true;
            int next = findNextSlotType(1);
            if (next == -1) next = findNextSlotType(2);
            if (next != -1) {
                currentSlot = next;
                if (!sorterBusy) {
                    final int slot = currentSlot;
                    sorterBusy = true;
                    sorter.setPosition(shootPositions[slot]);
                    new Thread(() -> {
                        try {
                            Thread.sleep(500);
                            slots[slot] = 0;
                        } catch (InterruptedException e) {
                            Thread.currentThread().interrupt();
                        } finally {
                            sorterBusy = false;
                        }
                    }).start();
                }
            }
        }
        if (!gamepad2.x) gp2_xPressed = false;

        //intake
        intakePower = -gamepad2.left_stick_y;
        intake.setPower(intakePower);

        telemetry.addData("=== DRIVETRAIN ===", "");
        if (currentPose != null) {
            telemetry.addData("Position", "X: %.1f, Y: %.1f", currentPose.getX(), currentPose.getY());
            telemetry.addData("Heading", "%.1f°", Math.toDegrees(currentPose.getHeading()));
            telemetry.addData("Speed Multiplier", "%.1fx", driveSpeedMultiplier);

            if (autoNavigating && currentTargetPose != null) {
                double distanceToTarget = Math.hypot(
                        currentPose.getX() - currentTargetPose.getX(),
                        currentPose.getY() - currentTargetPose.getY()
                );
                telemetry.addData("Target", "X:%.1f Y:%.1f", currentTargetPose.getX(), currentTargetPose.getY());
                telemetry.addData("Distance", "%.2f\"", distanceToTarget);
            }
        } else {
            telemetry.addData("Position", "INITIALIZING");
        }

        // === SLOT STATUS ===
        StringBuilder slotStatus = new StringBuilder();
        for (int i = 0; i < slots.length; i++) {
            String type;
            switch (slots[i]) {
                case 1: type = "pp"; break;
                case 2: type = "gr"; break;
                default: type = "nu"; break;
            }
            slotStatus.append(type);
            if (i < slots.length - 1) slotStatus.append(" | ");
        }
        telemetry.addData("Slots", slotStatus.toString());
        telemetry.addData("Current Slot", currentSlot);
        telemetry.addData("Loading", loading ? "YES" : "NO");

        telemetry.addData("Color", "R:%d G:%d B:%d", r, g, b);
        telemetry.addData("Detected (cur)", slots[currentSlot] == 1 ? "Purple" : slots[currentSlot] == 2 ? "Green" : "Unknown");
        if (autoNavigating) {
            long elapsedTime = System.currentTimeMillis() - autoNavStartTime;
            long remainingTime = (AUTO_NAV_TIMEOUT_MS - elapsedTime) / 1000;
            telemetry.addData("Auto Nav", "Following [%ds]", remainingTime);
        } else {
            telemetry.addData("Auto Nav", "Idle");
        }
        telemetry.addData("", "");

        telemetry.addData("=== OUTTAKE ===", "");
        telemetry.addData("Power", "%.2f", outtakePower);
        telemetry.addData("Speed", "%.2f", outtakeSpeed);
        telemetry.addData("", "");

        telemetry.addData("=== TURRET ===", "");
        telemetry.addData("Power", "%.2f", turretPower);
        telemetry.addData("", "");

        telemetry.addData("=== INTAKE ===", "");
        telemetry.addData("Power", "%.2f", intakePower);
        telemetry.addData("", "");

        telemetry.addData("=== SERVOS ===", "");
        telemetry.addData("binit", "%.2f", binit.getPosition());
        telemetry.addData("sorter", sorter.getPosition());

        telemetry.update();
    }

    private void startAutoNavigation(Pose currentPose, Pose target) {
        currentTargetPose = target;
        autoPath = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(currentPose.getX(), currentPose.getY(), currentPose.getHeading()),
                        target
                ))
                .setLinearHeadingInterpolation(currentPose.getHeading(), target.getHeading())
                .build();

        follower.followPath(autoPath, false);
        autoNavigating = true;
        autoNavStartTime = System.currentTimeMillis();
    }

    private void cancelAutoNavigation() {
        follower.breakFollowing();
        autoNavigating = false;
        currentTargetPose = null;
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    private void setManualDrive(double forward, double strafe, double turn) {
        double frontLeftPower = forward + strafe + turn;
        double frontRightPower = forward - strafe - turn;
        double backLeftPower = forward - strafe + turn;
        double backRightPower = forward + strafe - turn;

        double maxPower = Math.max(Math.abs(frontLeftPower), Math.max(Math.abs(frontRightPower),
                Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))));

        if (maxPower > 1.0) {
            frontLeftPower /= maxPower;
            frontRightPower /= maxPower;
            backLeftPower /= maxPower;
            backRightPower /= maxPower;
        }

        // Apply speed multiplier
        frontLeftPower *= driveSpeedMultiplier;
        frontRightPower *= driveSpeedMultiplier;
        backLeftPower *= driveSpeedMultiplier;
        backRightPower *= driveSpeedMultiplier;

        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
    }

    @Override
    public void stop() {
        outtake1.setPower(0);
        outtake2.setPower(0);
        turret.setPower(0);
        intake.setPower(0);
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    private int findNextSlot(int target) {
        for (int i = 0; i < slots.length; i++) {
            if (slots[i] == target) return i;
        }
        return -1;
    }

    private int findNextSlotWithBall() {
        for (int i = 0; i < slots.length; i++) {
            if (slots[i] != 0) return i;
        }
        return -1;
    }

    private int findNextSlotType(int type) {
        for (int i = 0; i < slots.length; i++) {
            if (slots[i] == type) return i;
        }
        return -1;
    }
}
