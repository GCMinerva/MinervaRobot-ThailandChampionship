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
import com.qualcomm.robotcore.hardware.ColorSensor;

@TeleOp(name="FullTeleFixed")
public class FullTeleFixed extends OpMode {
    boolean pressed = false;
    boolean aPressed = false;
    boolean bPressed = false;
    boolean xPressed = false;
    boolean x1Pressed = false;
    boolean yPressed = false;

    private Follower follower;
    private boolean sorterBusy = false;

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private ColorSensor colorSensor;
    private DcMotor outtake1, outtake2, intake, turret;
    private Servo binit, sorter;
    private double outtakePower = 0;
    private double outtakeSpeed = 0.5;
    private long lastSpeedChangeTime = 0;
    private double intakePower = 0;
    private double testpo = 0;

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

    private int sorterShootMode = 0;
    private int sorterInputMode = 0;

    private boolean autoNavigating = false;
    private boolean manualOverride = false;
    private PathChain autoPath;
    private long autoNavStartTime = 0;
    private final long AUTO_NAV_TIMEOUT_MS = 5000;

    private final Pose startPose = new Pose(58, 0, Math.toRadians(90));
    private final Pose targetPose = new Pose(71.7, 67.1, Math.toRadians(132.2));

    private final double POSITION_TOLERANCE_PERCENT = 0.05;
    private final double positionTolerance = Math.hypot(targetPose.getX(), targetPose.getY()) * POSITION_TOLERANCE_PERCENT;
    private final double HEADING_TOLERANCE_DEGREES = 5.0;

    // ===== Sorter system (3 ช่องเก็บบอล) =====
    // 0 = ว่าง, 1 = บอลม่วง, 2 = บอลเขียว
    int[] slots = {0, 0, 0};
    int currentSlot = 0;
    double[] loadPositions = {0.15, 0.533, 0.918};
    double[] shootPositions = {0.751, 0, 0.354};

    // loading state (non-blocking): เมื่อกดเก็บ ให้รอเวลาให้ servo หมุนเสร็จก่อนอ่านสี
    private boolean loading = false;
    private long loadStartTime = 0;
    private int loadingSlot = -1;
    private final int LOAD_DELAY_MS = 700; // ปรับตามความเร็ว servo/กลไก

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

        outtake1.setDirection(DcMotor.Direction.REVERSE);
        outtake2.setDirection(DcMotor.Direction.REVERSE);
        outtake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        binit = hardwareMap.get(Servo.class, "binit");
        sorter = hardwareMap.get(Servo.class, "sorter");

        binit.setPosition(1);
        sorter.setPosition(0.55);

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

        follower.update();
        Pose currentPose = follower.getPose();

        // อ่านค่าสีเพื่อ telemetry (ค่าสดของเฟรมนี้)
        int red = colorSensor.red();
        int green = colorSensor.green();
        int blue = colorSensor.blue();

        double forwardSpeed = -gamepad2.left_stick_y;
        double strafeSpeed = gamepad2.left_stick_x;
        double turnSpeed = gamepad2.right_stick_x;

        boolean joystickActive = Math.abs(forwardSpeed) > 0.1 || Math.abs(strafeSpeed) > 0.1 || Math.abs(turnSpeed) > 0.1;

        if (autoNavigating && (System.currentTimeMillis() - autoNavStartTime) > AUTO_NAV_TIMEOUT_MS) {
            cancelAutoNavigation();
            telemetry.addData("Auto Nav", "CANCELLED - timeout");
        }

        if (autoNavigating && joystickActive && !manualOverride) {
            manualOverride = true;
            follower.breakFollowing();
        } else if (autoNavigating && !joystickActive && manualOverride) {
            manualOverride = false;
            if (currentPose != null) {
                startAutoNavigation(currentPose);
            }
        }

        if (!autoNavigating || manualOverride) {
            setManualDrive(forwardSpeed, strafeSpeed, turnSpeed);
        }

        if (gamepad2.a && !gp2_aPressed && currentPose != null) {
            gp2_aPressed = true;
            startAutoNavigation(currentPose);
        }
        if (!gamepad2.a) gp2_aPressed = false;

        if (gamepad2.b && !gp2_bPressed) {
            gp2_bPressed = true;
            if (autoNavigating) {
                cancelAutoNavigation();
            }
        }
        if (!gamepad2.b) gp2_bPressed = false;

        if (autoNavigating && !follower.isBusy() && !manualOverride) {
            double distanceToTarget = Math.hypot(
                    currentPose.getX() - targetPose.getX(),
                    currentPose.getY() - targetPose.getY()
            );
            double headingError = Math.abs(Math.toDegrees(currentPose.getHeading() - targetPose.getHeading()));

            if (distanceToTarget <= positionTolerance && headingError <= HEADING_TOLERANCE_DEGREES) {
                cancelAutoNavigation();
            }
        }

        if (gamepad2.x && !gp2_xPressed) {
            gp2_xPressed = true;
            outtakePower = outtakeSpeed;
        }
        if (!gamepad2.x) gp2_xPressed = false;

        if (gamepad2.y && !gp2_yPressed) {
            gp2_yPressed = true;
            outtakePower = 0;
        }
        if (!gamepad2.y) gp2_yPressed = false;

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

        if (outtakePower > 0) {
            outtakePower = outtakeSpeed;
        }

        outtake1.setPower(outtakePower);
        outtake2.setPower(outtakePower);

        // Turret control with right joystick (gamepad2)
        double turretPower = gamepad2.right_stick_y;
        turret.setPower(turretPower);

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

        if (gamepad1.right_bumper && !gp1_rbPressed) {
            gp1_rbPressed = true;
            binit.setPosition(0.35);
            new Thread(() -> {
                try {
                    Thread.sleep(500);
                    binit.setPosition(1);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
            }).start();
        }
        if (!gamepad1.right_bumper) gp1_rbPressed = false;

        // === LOADING (non-blocking) ===
        if (gamepad1.y && !yPressed && !loading) {
            aPressed = true;
            int next = findNextSlot(0); // หาช่องว่าง
            if (next != -1) {
                currentSlot = next;
                sorter.setPosition(loadPositions[currentSlot]);

                // เริ่ม state loading: รอให้ servo หมุนเสร็จ แล้วค่อยอ่านสี
                loading = true;
                loadStartTime = System.currentTimeMillis();
                loadingSlot = currentSlot;
            }
        }
        if (!gamepad1.y) yPressed = false;

        // เมื่อ loading ผ่านเวลา LOAD_DELAY_MS ให้อ่านสีจริงและอัพเดต slots
        if (loading && (System.currentTimeMillis() - loadStartTime >= LOAD_DELAY_MS)) {
            // อ่านค่าสี ณ ตอนนี้ (สด)
            int r = colorSensor.red();
            int g = colorSensor.green();
            int b = colorSensor.blue();

            // ตรวจสี (threshold เหมือนตัวอย่างก่อนหน้า) — ปรับตามสนามจริงได้
            if (g > r && g > b && g > 3000) {
                slots[loadingSlot] = 2; // ม่วง
            } else if (r > 500 && b > 500 && green > 500) {
                slots[loadingSlot] = 1; // เขียว
            } else {
                slots[loadingSlot] = 0; // ไม่แน่ใจ
            }

            telemetry.addData("LoadSlot", loadingSlot);
            telemetry.addData("LoadColor(RGB)", "%d %d %d", r, g, b);

            // reset loading state
            loading = false;
            loadingSlot = -1;
        }

        if (gamepad1.b) { // Y+B ยิงบอลม่วง
            int next = findNextSlotType(1);
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
        if (gamepad1.a) { // Y+A ยิงบอลเขียว
            int next = findNextSlotType(2);
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

        // === intake toggle (X) ===
        if (gamepad1.x && !gp1_xPressed) {
            gp1_xPressed = true;
            x1Pressed = !x1Pressed;  // toggle เมื่อกดปุ่ม (ไม่ใช่ตอนปล่อย)

            if (x1Pressed) {
                intakePower = -0.75;  // เปิดดูด
            } else {
                intakePower = 0;      // ปิดดูด
            }
        }
        if (!gamepad1.x) {
            gp1_xPressed = false; // รีเซ็ตสถานะกันค้าง
        }

        intake.setPower(intakePower);

        telemetry.addData("=== DRIVETRAIN ===", "");
        if (currentPose != null) {
            telemetry.addData("Position", "X: %.1f, Y: %.1f", currentPose.getX(), currentPose.getY());
            telemetry.addData("Heading", "%.1f°", Math.toDegrees(currentPose.getHeading()));

            if (autoNavigating) {
                double distanceToTarget = Math.hypot(
                        currentPose.getX() - targetPose.getX(),
                        currentPose.getY() - targetPose.getY()
                );
                telemetry.addData("Distance", "%.2f\" (tol: %.2f\")", distanceToTarget, positionTolerance);
            }
        } else {
            telemetry.addData("Position", "INITIALIZING");
        }

        // === SLOT STATUS (แสดงทั้ง array) ===
        StringBuilder slotStatus = new StringBuilder();
        for (int i = 0; i < slots.length; i++) {
            String type;
            switch (slots[i]) {
                case 1: type = "pp"; break; // ม่วง
                case 2: type = "gr"; break; // เขียว
                default: type = "nu"; break; // ว่าง
            }
            slotStatus.append(type);
            if (i < slots.length - 1) slotStatus.append(" | ");
        }
        telemetry.addData("Slots", slotStatus.toString());
        telemetry.addData("Current Slot", currentSlot);
        telemetry.addData("Loading", loading ? "YES" : "NO");

        telemetry.addData("Color", "R:%d G:%d B:%d", red, green, blue);
        telemetry.addData("Detected (cur)", slots[currentSlot] == 1 ? "Purple" : slots[currentSlot] == 2 ? "Green" : "Unknown");
        if (autoNavigating) {
            long elapsedTime = System.currentTimeMillis() - autoNavStartTime;
            long remainingTime = (AUTO_NAV_TIMEOUT_MS - elapsedTime) / 1000;
            String status = manualOverride ? "Override" : "Following";
            telemetry.addData("Auto Nav", "%s [%ds]", status, remainingTime);
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
        telemetry.addData("sorter", "%.2f", sorter.getPosition());

        telemetry.update();
    }

    private void startAutoNavigation(Pose currentPose) {
        autoPath = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(currentPose.getX(), currentPose.getY(), currentPose.getHeading()),
                        targetPose
                ))
                .setLinearHeadingInterpolation(currentPose.getHeading(), targetPose.getHeading())
                .build();

        follower.followPath(autoPath, false);
        autoNavigating = true;
        autoNavStartTime = System.currentTimeMillis();
    }

    private void cancelAutoNavigation() {
        follower.breakFollowing();
        autoNavigating = false;
        manualOverride = false;
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