package org.firstinspires.ftc.teamcode.archived.opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.archived.subsystems.ActuatorSubsystem;
import org.firstinspires.ftc.teamcode.archived.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.archived.util.GlobalData;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.TeleOpBallDetector;

@TeleOp(name = "MinervaTeleOpV3BLUE")
public class MinervaTeleOpV3BLUE extends OpMode {
    private Follower follower;
    private ActuatorSubsystem actuator;
    private TurretSubsystem turret;
    private TeleOpBallDetector ballDetector;

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private Servo lifter, lifter1;
    private double speedMultiplier = 1.0;
    private double lifterPos = 1.0;
    private boolean gp1_dpad_up_pressed = false;
    private boolean liftAct = false;

    // Field-centric control
    private boolean fieldCentricEnabled = true;
    private double referenceHeading = Math.toRadians(180); // 180° is forward (Blue alliance)
    private boolean gp1_ltPressed = false;

    private boolean autoNavigating = false;
    private PathChain autoPath;
    private long autoNavStartTime = 0;
    private final long AUTO_NAV_TIMEOUT_MS = 5000;

    // RED GOAL
    // private final Pose startPose = new Pose(110, 129.5, Math.toRadians(90));
    // private final Pose targetPoseX = new Pose(80, 16, Math.toRadians(65));
    // private final Pose targetPoseY = new Pose(73.91865357643759,
    // 90.68162692847125, Math.toRadians(38));
    // private final Pose targetPoseA = new Pose(38.57503506311361, 39.382889200561,
    // Math.toRadians(90));

    // BLUE GOAL
    private final Pose startPose = new Pose(110, 129.5, Math.toRadians(90));
    private final Pose targetPoseX = new Pose(64, 16, Math.toRadians(65));
    private final Pose targetPoseY = new Pose(73.7, 90.1, Math.toRadians(142));
    private final Pose targetPoseA = new Pose(105.4249649368864, 35.20056100981767, Math.toRadians(90));
    private Pose currentTargetPose = null;

    private boolean gp1_yPressed = false;
    private boolean gp1_aPressed = false;
    private boolean gp1_bPressed = false;
    private boolean gp1_rbPressed = false;
    private boolean gp1_xPressed = false;
    private boolean gp1_dpad_left_pressed = false;
    private boolean turretTrackingEnabled = true;

    private int outtakeSorterPosition = 1;
    private boolean outtakeOn = false;
    private double outtakeSpeed = -0.63; // Default outtake speed
    private boolean gp2_ltPressed = false;
    private boolean gp2_rtPressed = false;
    private boolean gp2_lbPressed = false;
    private boolean gp2_rbPressed = false;
    private boolean gp2_dpad_up_pressed = false;
    private boolean gp2_dpad_down_pressed = false;
    private boolean gp2_dpad_left_pressed = false;
    private boolean gp2_dpad_right_pressed = false;
    private boolean autoBallDetectionEnabled = false;
    private boolean joystickIntakeOverride = false;

    // Right trigger cooldown timer
    private long lastShootTime = 0;
    private static final long SHOOT_COOLDOWN_MS = 1000; // 1.0 second cooldown

    // Smart sorter cycling variables
    private int ballCount = 0;
    private boolean[] slotFilled = { false, false, false }; // P1, P2, P3
    private int[] shootingOrder = new int[3]; // Dynamic shooting order
    private int shootingOrderLength = 0;
    private int currentShootIndex = 0;
    private boolean isFirstShotInSequence = true; // Track if this is the first shot

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);

        // Use autonomous end pose if available, otherwise use default startPose
        Pose initialPose = startPose;
        if (GlobalData.hasAutonomousRun && GlobalData.autonomousEndPose != null) {
            initialPose = GlobalData.autonomousEndPose;
            telemetry.addData("✅ Using Autonomous End Pose", "");
            telemetry.addData("X", "%.2f", initialPose.getX());
            telemetry.addData("Y", "%.2f", initialPose.getY());
            telemetry.addData("Heading", "%.2f°", Math.toDegrees(initialPose.getHeading()));
        } else {
            telemetry.addData("ℹ️ Using Default Start Pose", "");
        }
        telemetry.addData("Field-Centric Mode", fieldCentricEnabled ? "ENABLED" : "DISABLED");
        telemetry.addData("Reference Heading", "%.1f° (180° = forward)", Math.toDegrees(referenceHeading));
        telemetry.update();

        follower.setStartingPose(initialPose);
        actuator = new ActuatorSubsystem(hardwareMap, telemetry);
        turret = new TurretSubsystem(hardwareMap);
        turret.setGoalPosition(0, 144); // Blue goal position (index 1)
        ballDetector = new TeleOpBallDetector(actuator);

        actuator.openBin();

        frontLeft = hardwareMap.get(DcMotor.class, "fl");
        frontRight = hardwareMap.get(DcMotor.class, "fr");
        backLeft = hardwareMap.get(DcMotor.class, "bl");
        backRight = hardwareMap.get(DcMotor.class, "br");

        lifter = hardwareMap.get(Servo.class, "lifter");
        lifter1 = hardwareMap.get(Servo.class, "lifter1");

        lifter.setPosition(1);
        lifter1.setPosition(0);

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
    public void start() {
        actuator.openBin();
    }

    @Override
    public void loop() {
        follower.update();
        Pose currentPose = follower.getPose();

        if (currentPose != null && turretTrackingEnabled) {
            turret.update(currentPose);
        }

        // Gamepad 1 controls
        handleGamepad1(currentPose);

        // Gamepad 2 controls
        handleGamepad2();

        telemetry.addData("=== DRIVETRAIN ===", "");
        if (currentPose != null) {
            telemetry.addData("Position", "X: %.1f, Y: %.1f", currentPose.getX(), currentPose.getY());
            telemetry.addData("Heading", "%.1f°", Math.toDegrees(currentPose.getHeading()));
            telemetry.addData("Speed Multiplier", "%.1f", speedMultiplier);
            telemetry.addData("Drive Mode", fieldCentricEnabled ? "FIELD-CENTRIC" : "ROBOT-CENTRIC");
            if (fieldCentricEnabled) {
                double headingOffset = currentPose.getHeading() - referenceHeading;
                telemetry.addData("Heading Offset", "%.1f°", Math.toDegrees(headingOffset));
            }
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

        telemetry.addData("Auto Ball Detection", autoBallDetectionEnabled ? "Running" : "Idle");
        telemetry.addData("Outtake Speed", "%.2f", Math.abs(outtakeSpeed));
        telemetry.addData("Turret Offset", "%.1f°", turret.getAngleOffset());

        // Smart sorter status (show in shooting order: P2, P1, P3)
        String slotStatus = String.format("P2:%s P1:%s P3:%s",
                slotFilled[1] ? "●" : "○",
                slotFilled[0] ? "●" : "○",
                slotFilled[2] ? "●" : "○");
        telemetry.addData("Ball Count", ballCount);
        telemetry.addData("Slots (Shoot Order)", slotStatus);

        telemetry.update();
    }

    private void handleGamepad1(Pose currentPose) {
        double forwardSpeed = -gamepad1.left_stick_y;
        double strafeSpeed = gamepad1.left_stick_x;
        double turnSpeed = gamepad1.right_stick_x;

        boolean joystickActive = Math.abs(forwardSpeed) > 0.1 || Math.abs(strafeSpeed) > 0.1
                || Math.abs(turnSpeed) > 0.1;

        if (autoNavigating) {
            if (joystickActive || (System.currentTimeMillis() - autoNavStartTime) > AUTO_NAV_TIMEOUT_MS) {
                cancelAutoNavigation();
            }
        }

        // Left Trigger: Toggle field-centric mode
        if (gamepad1.left_trigger > 0.1 && !gp1_ltPressed) {
            gp1_ltPressed = true;
            fieldCentricEnabled = !fieldCentricEnabled;
        }
        if (gamepad1.left_trigger < 0.1)
            gp1_ltPressed = false;

        if (gamepad1.right_bumper && !gp1_rbPressed) {
            gp1_rbPressed = true;
            if (autoNavigating) {
                cancelAutoNavigation();
            }
        }
        if (!gamepad1.right_bumper)
            gp1_rbPressed = false;

        if (!autoNavigating) {
            setManualDrive(forwardSpeed, strafeSpeed, turnSpeed, currentPose);
        }

        if (gamepad1.y && !gp1_yPressed && currentPose != null) {
            gp1_yPressed = true;
            startAutoNavigation(currentPose, targetPoseY);
        }
        if (!gamepad1.y)
            gp1_yPressed = false;

        if (gamepad1.x && !gp1_xPressed && currentPose != null) {
            gp1_xPressed = true;
            startAutoNavigation(currentPose, targetPoseX);
        }
        if (!gamepad1.x)
            gp1_xPressed = false;

        if (gamepad1.a && !gp1_aPressed && currentPose != null) {
            gp1_aPressed = true;
            speedMultiplier = 0.4; // Set drive speed to low
            startAutoNavigation(currentPose, targetPoseA);
        }
        if (!gamepad1.a)
            gp1_aPressed = false;

        if (gamepad1.b && !gp1_bPressed) {
            gp1_bPressed = true;
            speedMultiplier = (speedMultiplier == 1.0) ? 0.5 : 1.0;
        }
        if (!gamepad1.b)
            gp1_bPressed = false;

        // Lifter control with dpad_up
        if (gamepad1.dpad_up && !gp1_dpad_up_pressed) {
            gp1_dpad_up_pressed = true;
            liftAct = !liftAct;

            if (liftAct) {
                lifterPos = 0; // Lift up
            } else {
                lifterPos = 1; // Lower down
            }
        }
        if (!gamepad1.dpad_up)
            gp1_dpad_up_pressed = false;

        // Clamp lifter position and update servos
        lifterPos = Range.clip(lifterPos, 0.7, 1);
        lifter.setPosition(lifterPos);
        lifter1.setPosition(1 - lifterPos - 0.05);

        // Turret tracking toggle with dpad_left
        if (gamepad1.dpad_left && !gp1_dpad_left_pressed) {
            gp1_dpad_left_pressed = true;
            turretTrackingEnabled = !turretTrackingEnabled;

            if (!turretTrackingEnabled) {
                // Disable tracking - turn turret to 0 degrees (robot heading)
                turret.turnToAngle(0);
            }
            // If enabling tracking, the turret.update() in loop() will handle it
        }
        if (!gamepad1.dpad_left)
            gp1_dpad_left_pressed = false;
    }

    private void handleGamepad2() {
        // D-pad for outtake speed control
        if (gamepad2.dpad_up && !gp2_dpad_up_pressed) {
            gp2_dpad_up_pressed = true;
            outtakeSpeed = Math.max(-1.0, outtakeSpeed - 0.01);
        }
        if (!gamepad2.dpad_up)
            gp2_dpad_up_pressed = false;

        if (gamepad2.dpad_down && !gp2_dpad_down_pressed) {
            gp2_dpad_down_pressed = true;
            outtakeSpeed = Math.min(0.0, outtakeSpeed + 0.01);
        }
        if (!gamepad2.dpad_down)
            gp2_dpad_down_pressed = false;

        // D-Pad Left: Decrease turret angle offset (subtract 1 degree)
        if (gamepad2.dpad_left && !gp2_dpad_left_pressed) {
            gp2_dpad_left_pressed = true;
            turret.addAngleOffset(-1.0);
        }
        if (!gamepad2.dpad_left)
            gp2_dpad_left_pressed = false;

        // D-Pad Right: Increase turret angle offset (add 1 degree)
        if (gamepad2.dpad_right && !gp2_dpad_right_pressed) {
            gp2_dpad_right_pressed = true;
            turret.addAngleOffset(1.0);
        }
        if (!gamepad2.dpad_right)
            gp2_dpad_right_pressed = false;

        // Left Trigger: Toggle Auto Ball Detection
        if (gamepad2.left_trigger > 0.1 && !gp2_ltPressed) {
            gp2_ltPressed = true;
            autoBallDetectionEnabled = !autoBallDetectionEnabled;
            joystickIntakeOverride = false; // Reset override on toggle
            if (autoBallDetectionEnabled) {
                ballDetector.start();
            } else {
                ballDetector.stop();
            }
        }
        if (gamepad2.left_trigger < 0.1)
            gp2_ltPressed = false;

        if (autoBallDetectionEnabled) {
            ballDetector.update();
            // Sync ball count from auto detector (cap at 3 max)
            int detectedCount = actuator.getBallCount();
            if (detectedCount != ballCount) {
                ballCount = Math.min(3, detectedCount); // Cap at 3 balls maximum
                updateSlotStatus(ballCount);
            }
        }

        // Right Trigger: Smart Cycle Sorter - only shoot from filled slots
        if (gamepad2.right_trigger > 0.1 && !gp2_rtPressed) {
            gp2_rtPressed = true;

            // Check cooldown timer
            long currentTime = System.currentTimeMillis();
            long timeSinceLastShot = currentTime - lastShootTime;

            if (timeSinceLastShot < SHOOT_COOLDOWN_MS) {
                // Still in cooldown period - ignore trigger press
                long remainingCooldown = (SHOOT_COOLDOWN_MS - timeSinceLastShot) / 1000;
                telemetry.addData("⏱️ Cooldown", "%ds remaining", remainingCooldown);
                telemetry.update();
                return;
            }

            // Check if there are any balls to shoot
            if (ballCount == 0) {
                // No balls to shoot - do nothing
                return;
            }

            // Get next filled slot from shooting order
            int nextSlot = getNextFilledSlot();

            if (nextSlot != -1) {
                // Check if outtake is already running
                boolean outtakeWasOn = outtakeOn;

                outtakeOn = true;

                // Disable auto ball detection
                if (autoBallDetectionEnabled) {
                    autoBallDetectionEnabled = false;
                    ballDetector.stop();
                }
                joystickIntakeOverride = false;

                // Set sorter to the next filled slot
                switch (nextSlot) {
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

                // Add delay if: (first shot with 1-2 balls) OR (outtake wasn't already on)
                // No delay for 3 balls
                if ((isFirstShotInSequence && (ballCount == 1 || ballCount == 2)) || !outtakeWasOn) {
                    try {
                        Thread.sleep(500); // 0.5 second delay
                    } catch (InterruptedException e) {
                        Thread.currentThread().interrupt();
                    }
                } else {
                    try {
                        Thread.sleep(250); // 0.25 second delay
                    } catch (InterruptedException e) {
                        Thread.currentThread().interrupt();
                    }
                }

                actuator.singleShotSequence();

                // Mark that we've done the first shot
                isFirstShotInSequence = false;

                // Decrease ball count after shooting
                ballCount = Math.max(0, ballCount - 1);

                // Mark the specific slot as empty
                slotFilled[nextSlot - 1] = false;

                // Update last shoot time for cooldown
                lastShootTime = currentTime;
            }
        }
        if (gamepad2.right_trigger < 0.1)
            gp2_rtPressed = false;

        // Left Bumper: Toggle Outtake
        if (gamepad2.left_bumper && !gp2_lbPressed) {
            gp2_lbPressed = true;
            outtakeOn = !outtakeOn;
            if (outtakeOn) {
                actuator.runOuttake(outtakeSpeed);
            } else {
                actuator.stopOuttake();
            }
        }
        if (!gamepad2.left_bumper)
            gp2_lbPressed = false;

        // if outtake is on, keep setting the power to the current speed
        if (outtakeOn) {
            actuator.runOuttake(outtakeSpeed);
        }

        // Left Joystick: Control Intake
        if (Math.abs(gamepad2.left_stick_y) > 0.5) { // Joystick is being used
            joystickIntakeOverride = true; // Latch the override
            if (gamepad2.left_stick_y < -0.5) {
                actuator.setIntakePower(1.0);
            } else {
                actuator.setIntakePower(-1.0);
            }
        } else { // Joystick is neutral
            if (autoBallDetectionEnabled && !joystickIntakeOverride) {
                actuator.setIntakePower(-1.0); // Auto mode runs intake
            } else {
                actuator.stopIntake(); // Stop if joystick is neutral and not in auto, or if override is active
            }
        }
    }

    private void startAutoNavigation(Pose currentPose, Pose target) {
        currentTargetPose = target;
        autoPath = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(currentPose.getX(), currentPose.getY(), currentPose.getHeading()),
                        currentTargetPose))
                .setLinearHeadingInterpolation(currentPose.getHeading(), currentTargetPose.getHeading())
                .build();

        follower.setMaxPower(1.0);
        follower.followPath(autoPath, false);
        autoNavigating = true;
        autoNavStartTime = System.currentTimeMillis();
    }

    private void cancelAutoNavigation() {
        follower.breakFollowing();
        follower.setMaxPower(1.0); // Reset to default
        autoNavigating = false;
        currentTargetPose = null;
        setManualDrive(0, 0, 0, follower.getPose());
    }

    private void setManualDrive(double forward, double strafe, double turn, Pose currentPose) {
        double frontLeftPower, frontRightPower, backLeftPower, backRightPower;

        if (fieldCentricEnabled && currentPose != null) {
            // Field-centric mode: rotate inputs by heading offset from 180°
            double botHeading = currentPose.getHeading() - referenceHeading;

            // Rotate the movement direction counter to the bot's rotation (GM0 standard)
            double rotX = strafe * Math.cos(-botHeading) - forward * Math.sin(-botHeading);
            double rotY = strafe * Math.sin(-botHeading) + forward * Math.cos(-botHeading);

            // Calculate motor powers with normalization
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(turn), 1);
            frontLeftPower = ((rotY + rotX + turn) / denominator) * speedMultiplier;
            frontRightPower = ((rotY - rotX - turn) / denominator) * speedMultiplier;
            backLeftPower = ((rotY - rotX + turn) / denominator) * speedMultiplier;
            backRightPower = ((rotY + rotX - turn) / denominator) * speedMultiplier;
        } else {
            // Robot-centric mode: use raw inputs
            frontLeftPower = (forward + strafe + turn) * speedMultiplier;
            frontRightPower = (forward - strafe - turn) * speedMultiplier;
            backLeftPower = (forward - strafe + turn) * speedMultiplier;
            backRightPower = (forward + strafe - turn) * speedMultiplier;

            // Normalize for robot-centric mode
            double maxPower = Math.max(Math.abs(frontLeftPower), Math.max(Math.abs(frontRightPower),
                    Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))));

            if (maxPower > 1.0) {
                frontLeftPower /= maxPower;
                frontRightPower /= maxPower;
                backLeftPower /= maxPower;
                backRightPower /= maxPower;
            }
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
            actuator.stopShooter();
        }
        if (turret != null) {
            turret.stop();
        }
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    // ===== Smart Sorter Cycling Helper Methods =====

    /**
     * Update slot status based on ball count
     * 1 ball -> P1 filled
     * 2 balls -> P1, P2 filled
     * >= 3 balls -> P1, P2, P3 filled
     */
    private void updateSlotStatus(int count) {
        // Reset all slots
        slotFilled[0] = false;
        slotFilled[1] = false;
        slotFilled[2] = false;

        // Fill slots based on count
        if (count >= 1) {
            slotFilled[0] = true; // P1
        }
        if (count >= 2) {
            slotFilled[1] = true; // P2
        }
        if (count >= 3) {
            slotFilled[2] = true; // P3
        }

        // Rebuild shooting order
        buildShootingOrder();

        // Reset first shot flag when new balls are detected
        isFirstShotInSequence = true;
    }

    /**
     * Build shooting order based on filled slots
     * >= 3 balls: P2 -> P1 -> P3
     * 2 balls: P2 -> P1
     * 1 ball: P1
     */
    private void buildShootingOrder() {
        shootingOrderLength = 0;
        currentShootIndex = 0;

        if (ballCount >= 3) {
            // Order: P2, P1, P3
            shootingOrder[0] = 2;
            shootingOrder[1] = 1;
            shootingOrder[2] = 3;
            shootingOrderLength = 3;
        } else if (ballCount == 2) {
            // Order: P2, P1
            shootingOrder[0] = 2;
            shootingOrder[1] = 1;
            shootingOrderLength = 2;
        } else if (ballCount == 1) {
            // Order: P1
            shootingOrder[0] = 1;
            shootingOrderLength = 1;
        }
    }

    /**
     * Get next filled slot from shooting order
     * 
     * @return slot number (1, 2, or 3) or -1 if no balls
     */
    private int getNextFilledSlot() {
        if (ballCount == 0 || shootingOrderLength == 0) {
            return -1;
        }

        if (currentShootIndex >= shootingOrderLength) {
            currentShootIndex = 0; // Wrap around
        }

        int slot = shootingOrder[currentShootIndex];
        currentShootIndex++;
        return slot;
    }

    /**
     * Mark a slot as empty after shooting
     * 
     * @param slotNumber 1, 2, or 3
     */
    private void markSlotEmpty(int slotNumber) {
        if (slotNumber >= 1 && slotNumber <= 3) {
            slotFilled[slotNumber - 1] = false;
            ballCount = Math.max(0, ballCount - 1);

            // Save current shoot index before updating
            int savedShootIndex = currentShootIndex;

            // Update slot status (this rebuilds shooting order)
            updateSlotStatus(ballCount);

            // Restore shoot index if we still have balls
            // This ensures we continue the cycle instead of restarting
            if (ballCount > 0 && savedShootIndex < shootingOrderLength) {
                currentShootIndex = savedShootIndex;
            }
        }
    }
}