package org.firstinspires.ftc.teamcode.archived.opmodes;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

@TeleOp(name="Tele2")
public class Tele2 extends OpMode {
    boolean pressed = false;
    private long pulseEndTime = 0;
    private boolean pulsing = false;

    private Pose currentTargetPose = null;
    boolean lbPressed = false;
    boolean shooterOn = false;
    boolean liftAct = false;
    boolean aPressed = false;
    boolean bPressed = false;
    boolean xPressed = false;
    boolean x1Pressed = false;
    boolean yPressed = false;
    boolean rbPressed = false;
    boolean rtPressed = false;
    boolean upPressed = false;

    boolean isSlowMode = false;

    private Follower follower;
    private boolean sorterBusy = false;
    private double lifterPos = 1.0;
    private boolean dpadPressed = false;

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private ColorSensor colorSensor;
    private DcMotor outtake1, outtake2, intake, turret;

    private Servo binit, sorter, binit1, lifter, lifter1;
    private double outtakePower = 0;
    private double outtakeSpeed = 0.75;
    private long lastSpeedChangeTime = 0;
    private double intakePower = 0;
    private double testpo = 0;

    private boolean gp2_xPressed = false;
    private boolean gp2_yPressed = false;
    private boolean gp2_aPressed = false;
    private boolean gp2_bPressed = false;
    private boolean gp1_yPressed = false;
    private boolean gp1_xPressed = false;
    private boolean gp1_aPressed = false;
    private boolean gp1_bPressed = false;
    private boolean gp1_rbPressed = false;
    private boolean gp1_lbPressed = false;
    private boolean goToParking = false;

    private int sorterShootMode = 0;
    private double speed = 0.8;
    private int sorterInputMode = 0;

    private boolean autoNavigating = false;
    private boolean manualOverride = false;
    private PathChain autoPath;
    private long autoNavStartTime = 0;
    private final long AUTO_NAV_TIMEOUT_MS = 5000;

    private final Pose startPose = new Pose(57.22, 10, Math.toRadians(90));
    private final Pose targetPose = new Pose(71.7, 67.1, Math.toRadians(132.2));
    private final Pose targetPoseX = new Pose(102, 35.5, Math.toRadians(0));


    private final double POSITION_TOLERANCE_PERCENT = 0.05;
    private final double positionTolerance = Math.hypot(targetPose.getX(), targetPose.getY()) * POSITION_TOLERANCE_PERCENT;
    private final double HEADING_TOLERANCE_DEGREES = 5.0;

    // ===== Sorter system (3 ช่องเก็บบอล) =====
    // 0 = ว่าง, 1 = บอลม่วง, 2 = บอลเขียว
    int[] slots = {0, 0, 0};
    int currentSlot = 0;
    double[] loadPositions = {0.15, 0.533, 0.918};
    double[] shootPositions = {0.751, 0, 0.354};

    int loadIndex = 0;
    int shootIndex = 0;

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
        outtake2.setDirection(DcMotor.Direction.FORWARD);
        outtake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lifter = hardwareMap.get(Servo.class, "lifter");
        lifter1 = hardwareMap.get(Servo.class, "lifter1");

        binit1 = hardwareMap.get(Servo.class, "brinit");
        binit = hardwareMap.get(Servo.class, "binit");
        sorter = hardwareMap.get(Servo.class, "sorter");

        lifter.setPosition(1);
        lifter1.setPosition(0);
        binit.setPosition(0.1);
        binit1.setPosition(0.8);
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

        if (gamepad1.a && !gp1_aPressed && currentPose != null) {
            gp1_aPressed = true;
            currentTargetPose = targetPose;
            startAutoNavigation(currentPose);
        }
        if (!gamepad1.a) gp1_aPressed = false;

        // === NEW: auto navigation (gamepad1.x) . parking zone ===
        if (gamepad1.x && !gp1_xPressed && currentPose != null) {
            gp1_xPressed = true;
            currentTargetPose = targetPoseX;
            goToParking = true; //indicating lifter servo
            startAutoNavigationToPose(currentPose, targetPoseX);
        }
        if (!gamepad1.x) gp1_xPressed = false;

        if (gamepad1.right_bumper && !gp1_rbPressed) {
            gp1_rbPressed = true;
            if (autoNavigating) {
                cancelAutoNavigation();
            }
        }
        if (!gamepad1.right_bumper) gp1_rbPressed = false;

        // robot speed
        if (gamepad1.b && !gp1_bPressed) {
            gp1_bPressed = true;
            isSlowMode = !isSlowMode;
        }
        if (!gamepad1.b) gp1_bPressed = false;
        speed = isSlowMode ? 0.4 : 0.8;

        // == SHORT BURST CONTROL ==
        if (gamepad1.dpad_up) {
            movePulse(0.4, 0, 0);   // เดินหน้า
        }
        else if (gamepad1.dpad_down) {
            movePulse(-0.4, 0, 0);  // ถอยหลัง
        }
        else if (gamepad1.dpad_left) {
            movePulse(0, -0.4, 0);  // เลี้ยว/ไปซ้าย
        }
        else if (gamepad1.dpad_right) {
            movePulse(0, 0.4, 0);   // เลี้ยว/ไปขวา
        }


        if (autoNavigating && !follower.isBusy() && !manualOverride && currentTargetPose != null) {

            double distanceToTarget = Math.hypot(
                    currentPose.getX() - currentTargetPose.getX(),
                    currentPose.getY() - currentTargetPose.getY()
            );

            double headingError = Math.abs(
                    Math.toDegrees(currentPose.getHeading() - currentTargetPose.getHeading())
            );

            if (distanceToTarget <= positionTolerance &&
                    headingError <= HEADING_TOLERANCE_DEGREES) {
                cancelAutoNavigation();
            }
        }

        if (gamepad2.left_bumper && !lbPressed) {
            lbPressed = true;
            shooterOn = !shooterOn;

            if (shooterOn) {
                outtakePower = outtakeSpeed;   // เปิด shooter
            } else {
                outtakePower = 0;              // ปิด shooter
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

        if (gamepad2.dpad_up && !upPressed) {
            upPressed = true;
            liftAct = !liftAct;

            if (liftAct) {
                lifterPos = 0;
            } else {
                lifterPos = 1;
            }
        }
        if (!gamepad2.dpad_up) upPressed = false;

        // จำกัดไม่ให้เกินช่วง 0–1
        lifterPos = Range.clip(lifterPos, 0, 1);

        // ส่งค่ากลับไปที่เซอร์โว
        lifter.setPosition(lifterPos);
        lifter1.setPosition(1-lifterPos);

//        long currentTime = System.currentTimeMillis();
//        if (currentTime - lastSpeedChangeTime >= 250) {
//            if (gamepad2.right_bumper) {
//                outtakeSpeed = Math.min(1.0, outtakeSpeed + 0.05);
//                lastSpeedChangeTime = currentTime;
//            }
//            if (gamepad2.right_trigger > 0.1) {
//                outtakeSpeed = Math.max(0.0, outtakeSpeed - 0.05);
//                lastSpeedChangeTime = currentTime;
//            }
//        }

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
            binit.setPosition(0.56);
            binit1.setPosition(0.34);
            new Thread(() -> {
                try {
                    Thread.sleep(500);
                    binit.setPosition(0.1);
                    binit1.setPosition(0.8);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
            }).start();
        }
        if (!gamepad2.b) gp2_bPressed = false;

        // ====== press y to load balls ======
        if (gamepad2.y && !yPressed) {
            yPressed = true;

            if (!sorterBusy) {
                sorterBusy = true;

                loadIndex = (loadIndex + 1) % loadPositions.length;
                sorter.setPosition(loadPositions[loadIndex]);

                new Thread(() -> {
                    try {
                        Thread.sleep(300);
                    } catch (InterruptedException e) {
                        Thread.currentThread().interrupt();
                    } finally {
                        sorterBusy = false;
                    }
                }).start();
            }
        }
        if (!gamepad2.y) yPressed = false;


        // ====== x for set up to shoot ======
        if (gamepad2.x && !xPressed) {
            xPressed = true;

            if (!sorterBusy) {
                sorterBusy = true;

                shootIndex = (shootIndex + 1) % shootPositions.length;
                sorter.setPosition(shootPositions[shootIndex]);

                new Thread(() -> {
                    try {
                        Thread.sleep(300);
                    } catch (InterruptedException e) {
                        Thread.currentThread().interrupt();
                    } finally {
                        sorterBusy = false;
                    }
                }).start();
            }
        }
        if (!gamepad2.x) xPressed = false;

        // Stop pulse automatically
        if (pulsing && System.currentTimeMillis() > pulseEndTime) {
            pulsing = false;
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
        }

        //intake
        intakePower = -gamepad2.left_stick_y;
        if (intakePower < 0) intakePower = -1;
        else if (intakePower > 0) intakePower = 1;
        intake.setPower(intakePower);

        telemetry.addData("=== DRIVETRAIN ===", "");
        if (currentPose != null) {
            telemetry.addData("Position", "X: %.1f, Y: %.1f", currentPose.getX(), currentPose.getY());
            telemetry.addData("Heading", "%.1f°", Math.toDegrees(currentPose.getHeading()));

            if (autoNavigating) {
                double distanceToTarget = Math.hypot(
                        currentPose.getX() - currentTargetPose.getX(),
                        currentPose.getY() - currentTargetPose.getY()
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

        telemetry.addData("Color", "R:%d G:%d B:%d", r, g, b);
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
        telemetry.addData("sorter", sorter.getPosition());

        telemetry.addData("Lifter Pos", lifter.getPosition());
        telemetry.addData("Lifter1 Pos", lifter1.getPosition());

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

    private void startAutoNavigationToPose(Pose currentPose, Pose target) {
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
        currentTargetPose = null;
        autoNavigating = false;
        manualOverride = false;

        follower.breakFollowing();
        autoNavigating = false;
        manualOverride = false;
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        // lift the robot while at the parking zone
        if (goToParking) {
            lifter.setPosition(1-lifter.getPosition());
            lifter1.setPosition(1-lifter1.getPosition());
            goToParking = false;
        }
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

        frontLeft.setPower(frontLeftPower * speed);
        frontRight.setPower(frontRightPower * speed);
        backLeft.setPower(backLeftPower * speed);
        backRight.setPower(backRightPower * speed);
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
    private void movePulse(double forward, double strafe, double turn) {
        double fl = forward + strafe + turn;
        double fr = forward - strafe - turn;
        double bl = forward - strafe + turn;
        double br = forward + strafe - turn;

        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);

        pulsing = true;
        pulseEndTime = System.currentTimeMillis() + 60;  // 120 ms
    }
}