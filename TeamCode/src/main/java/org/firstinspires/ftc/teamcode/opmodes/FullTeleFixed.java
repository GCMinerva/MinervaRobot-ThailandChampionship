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

@TeleOp(name="FullTeleFixed")
public class FullTeleFixed extends OpMode {

    private Follower follower;
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotor outtake, intake;
    private Servo binit, sorter;

    private double outtakePower = 0;
    private double outtakeSpeed = 0.5;
    private long lastSpeedChangeTime = 0;
    private double intakePower = 0;

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

        outtake = hardwareMap.get(DcMotor.class, "outtake");
        intake = hardwareMap.get(DcMotor.class, "intake");

        outtake.setDirection(DcMotor.Direction.FORWARD);
        outtake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        binit = hardwareMap.get(Servo.class, "binit");
        sorter = hardwareMap.get(Servo.class, "sorter");

        binit.setPosition(0.35);
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

        outtake.setPower(outtakePower);

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

        if (gamepad1.a && !gp1_aPressed) {
            gp1_aPressed = true;
            sorterInputMode = (sorterInputMode + 1) % 3;
            switch (sorterInputMode) {
                case 0: sorter.setPosition(0.0); break;
                case 1: sorter.setPosition(0.375); break;
                case 2: sorter.setPosition(0.75); break;
            }
        }
        if (!gamepad1.a) gp1_aPressed = false;

        if (gamepad1.b && !gp1_bPressed) {
            gp1_bPressed = true;
            sorterShootMode = (sorterShootMode + 1) % 3;
            switch (sorterShootMode) {
                case 0: sorter.setPosition(0.55); break;
                case 1: sorter.setPosition(0.9); break;
                case 2: sorter.setPosition(0.15); break;
            }
        }
        if (!gamepad1.b) gp1_bPressed = false;

        if (gamepad1.x && !gp1_xPressed) {
            gp1_xPressed = true;
            intakePower = -0.75;
        }
        if (!gamepad1.x) gp1_xPressed = false;

        if (gamepad1.y && !gp1_yPressed) {
            gp1_yPressed = true;
            intakePower = 0;
        }
        if (!gamepad1.y) gp1_yPressed = false;

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
        outtake.setPower(0);
        intake.setPower(0);
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
}
