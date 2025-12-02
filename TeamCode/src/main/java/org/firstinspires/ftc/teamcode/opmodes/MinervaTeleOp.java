package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.util.Range;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.archived.subsystems.ActuatorSubsystem;
import org.firstinspires.ftc.teamcode.archived.subsystems.AprilTagSubsystem;
import org.firstinspires.ftc.teamcode.archived.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.qualcomm.robotcore.hardware.ColorSensor;

@TeleOp(name="MinervaTeleOp V2")
public class MinervaTeleOp extends OpMode {
    private Follower follower;
    private ActuatorSubsystem actuator;

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private double speedMultiplier = 1.0;

    private boolean autoNavigating = false;
    private PathChain autoPath;
    private long autoNavStartTime = 0;
    private final long AUTO_NAV_TIMEOUT_MS = 5000;

    private final Pose startPose = new Pose(58, 0, Math.toRadians(90));
    private final Pose targetPoseX = new Pose(14, 135, Math.toRadians(110));
    private final Pose targetPoseY = new Pose(71.7, 67.1, Math.toRadians(132.2));
    private final Pose targetPoseA = new Pose(39, 34, Math.toRadians(90));
    private Pose currentTargetPose = null;

    private boolean gp1_yPressed = false;
    private boolean gp1_aPressed = false;
    private boolean gp1_bPressed = false;
    private boolean gp1_rbPressed = false;
    private boolean gp1_xPressed = false;

    private int intakeSorterPosition = 1;
    private int outtakeSorterPosition = 1;
    private boolean outtakeOn = false;
    private boolean gp2_ltPressed = false;
    private boolean gp2_rtPressed = false;
    private boolean gp2_lbPressed = false;
    private boolean gp2_rbPressed = false;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        actuator = new ActuatorSubsystem(hardwareMap, telemetry);

        frontLeft = hardwareMap.get(DcMotor.class, "fl");
        frontRight = hardwareMap.get(DcMotor.class, "fr");
        backLeft = hardwareMap.get(DcMotor.class, "bl");
        backRight = hardwareMap.get(DcMotor.class, "br");

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

        // Gamepad 1 controls
        handleGamepad1(currentPose);

        // Gamepad 2 controls
        handleGamepad2();

        telemetry.addData("=== DRIVETRAIN ===", "");
        if (currentPose != null) {
            telemetry.addData("Position", "X: %.1f, Y: %.1f", currentPose.getX(), currentPose.getY());
            telemetry.addData("Heading", "%.1f°", Math.toDegrees(currentPose.getHeading()));
            telemetry.addData("Speed Multiplier", "%.1f", speedMultiplier);
        } else {
            telemetry.addData("Position", "INITIALIZING");
        }

        if (autoNavigating) {
            long elapsedTime = System.currentTimeMillis() - autoNavStartTime;
            long remainingTime = (AUTO_NAV_TIMEOUT_MS - elapsedTime) / 1000;
            telemetry.addData("Auto Nav", "Following [%ds]", remainingTime);
        } else {
            telemetry.addData("Auto Nav", "Idle");
        }
        telemetry.update();
    }

    private void handleGamepad1(Pose currentPose) {
        double forwardSpeed = -gamepad1.left_stick_y;
        double strafeSpeed = gamepad1.left_stick_x;
        double turnSpeed = gamepad1.right_stick_x;

        boolean joystickActive = Math.abs(forwardSpeed) > 0.1 || Math.abs(strafeSpeed) > 0.1 || Math.abs(turnSpeed) > 0.1;

        if (autoNavigating) {
            if (joystickActive || (System.currentTimeMillis() - autoNavStartTime) > AUTO_NAV_TIMEOUT_MS) {
                cancelAutoNavigation();
            }
        }

        if (gamepad1.right_bumper && !gp1_rbPressed) {
            gp1_rbPressed = true;
            if (autoNavigating) {
                cancelAutoNavigation();
            }
        }
        if (!gamepad1.right_bumper) gp1_rbPressed = false;

        if (!autoNavigating) {
            setManualDrive(forwardSpeed, strafeSpeed, turnSpeed);
        }

        if (gamepad1.y && !gp1_yPressed && currentPose != null) {
            gp1_yPressed = true;
            startAutoNavigation(currentPose, targetPoseY);
        }
        if (!gamepad1.y) gp1_yPressed = false;

        if (gamepad1.x && !gp1_xPressed && currentPose != null) {
            gp1_xPressed = true;
            startAutoNavigation(currentPose, targetPoseX);
        }
        if (!gamepad1.x) gp1_xPressed = false;

        if (gamepad1.a && !gp1_aPressed && currentPose != null) {
            gp1_aPressed = true;
            startAutoNavigation(currentPose, targetPoseA);
        }
        if (!gamepad1.a) gp1_aPressed = false;

        if (gamepad1.b && !gp1_bPressed) {
            gp1_bPressed = true;
            speedMultiplier = (speedMultiplier == 1.0) ? 0.5 : 1.0;
        }
        if (!gamepad1.b) gp1_bPressed = false;
    }

    private void handleGamepad2() {
        // Left Trigger: Cycle Sorter Intake Position
        if (gamepad2.left_trigger > 0.1 && !gp2_ltPressed) {
            gp2_ltPressed = true;
            intakeSorterPosition++;
            if (intakeSorterPosition > 3) {
                intakeSorterPosition = 1;
            }
            switch (intakeSorterPosition) {
                case 1:
                    actuator.setSorterIN1();
                    break;
                case 2:
                    actuator.setSorterIN2();
                    break;
                case 3:
                    actuator.setSorterIN3();
                    break;
            }
        }
        if (gamepad2.left_trigger < 0.1) gp2_ltPressed = false;

        // Right Trigger: Cycle Sorter Outtake Position
        if (gamepad2.right_trigger > 0.1 && !gp2_rtPressed) {
            gp2_rtPressed = true;
            outtakeSorterPosition++;
            if (outtakeSorterPosition > 3) {
                outtakeSorterPosition = 1;
            }
            switch (outtakeSorterPosition) {
                case 1:
                    actuator.setSorterP1();
                    break;
                case 2:
                    actuator.setSorterP2();
                    break;
                case 3:
                    actuator.setSorterP3();
                    break;
            }
        }
        if (gamepad2.right_trigger < 0.1) gp2_rtPressed = false;

        // Left Bumper: Toggle Outtake
        if (gamepad2.left_bumper && !gp2_lbPressed) {
            gp2_lbPressed = true;
            outtakeOn = !outtakeOn;
            if (outtakeOn) {
                actuator.runOuttake(1.0); // Or use a variable power
            } else {
                actuator.stopOuttake();
            }
        }
        if (!gamepad2.left_bumper) gp2_lbPressed = false;

        // Right Bumper: Feed Ball
        if (gamepad2.right_bumper && !gp2_rbPressed) {
            gp2_rbPressed = true;
            actuator.feedBall(0.5);
        }
        if (!gamepad2.right_bumper) gp2_rbPressed = false;

        // Left Joystick: Control Intake
        if (gamepad2.left_stick_y < -0.5) { // Joystick pushed up
            actuator.setIntakePower(1.0);
        } else if (gamepad2.left_stick_y > 0.5) { // Joystick pushed down
            actuator.setIntakePower(-1.0);
        } else {
            actuator.stopIntake();
        }
    }

    private void startAutoNavigation(Pose currentPose, Pose target) {
        currentTargetPose = target;
        autoPath = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(currentPose.getX(), currentPose.getY(), currentPose.getHeading()),
                        currentTargetPose
                ))
                .setLinearHeadingInterpolation(currentPose.getHeading(), currentTargetPose.getHeading())
                .build();

        follower.followPath(autoPath, false);
        autoNavigating = true;
        autoNavStartTime = System.currentTimeMillis();
    }

    private void cancelAutoNavigation() {
        follower.breakFollowing();
        autoNavigating = false;
        currentTargetPose = null;
        setManualDrive(0,0,0);
    }

    private void setManualDrive(double forward, double strafe, double turn) {
        double frontLeftPower = (forward + strafe + turn) * speedMultiplier;
        double frontRightPower = (forward - strafe - turn) * speedMultiplier;
        double backLeftPower = (forward - strafe + turn) * speedMultiplier;
        double backRightPower = (forward + strafe - turn) * speedMultiplier;

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
        if (actuator != null) {
            actuator.stopIntake();
            actuator.stopOuttake();
        }
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
}