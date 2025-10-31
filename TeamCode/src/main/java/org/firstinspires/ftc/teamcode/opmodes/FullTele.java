package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp(name="FullTele")
public class FullTele extends LinearOpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight, Control, intake;
    private Servo myServo, axon;

    double speed = -0.8;
    double level = 0;
    double pos = 0;
    double power = 0;

    // Axon servo control variables
    int inputMode = 0;  // 0 = initial, 1 = first tray, 2 = second tray, 3 = third tray
    int shootMode = 0;  // 0 = initial, 1 = first shoot, 2 = second shoot, 3 = third shoot
    double outtakePower = 0;
    double outtakeSpeed = 0.5;  // Default outtake speed
    long lastSpeedChangeTime = 0;  // Track last time speed was changed

    @Override
    public void runOpMode() {

        frontLeft  = hardwareMap.get(DcMotor.class, "fl");
        frontRight = hardwareMap.get(DcMotor.class, "fr");
        backLeft   = hardwareMap.get(DcMotor.class, "bl");
        backRight  = hardwareMap.get(DcMotor.class, "br");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
//        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        // Optional safety
        for (DcMotor m : new DcMotor[]{frontLeft, frontRight, backLeft, backRight}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        Control = hardwareMap.get(DcMotor.class, "outtake");
        intake = hardwareMap.get(DcMotor.class, "intake");

        // Map servo
        myServo = hardwareMap.get(Servo.class, "binit");
        axon = hardwareMap.get(Servo.class, "sorter");  // Axon servo mapped to "sorter"

        Control.setDirection(DcMotor.Direction.FORWARD);
        myServo.setPosition(0.35);  // Initialize binit to 0.75
        axon.setPosition(0.0);  // Initialize axon to 0

        waitForStart();

        boolean pressed = false;
        boolean aPressed = false;
        boolean bPressed = false;
        boolean xPressed = false;
        boolean yPressed = false;

        while (opModeIsActive()) {
            // A button: Input mode control (tray positions)
            if (gamepad1.a && !aPressed) {
                aPressed = true;
                inputMode = (inputMode + 1) % 4;  // Cycle through 0 -> 1 -> 2 -> 3 -> 0

                switch (inputMode) {
                    case 0:
                        axon.setPosition(0.0);  // Initial position - let ball in
                        break;
                    case 1:
                        axon.setPosition(0.0);  // Still at 0, waiting for ball detection
                        break;
                    case 2:
                        axon.setPosition(0.4);  // Second tray
                        break;
                    case 3:
                        axon.setPosition(0.75); // Third tray
                        break;
                }
            }
            if (!gamepad1.a) aPressed = false;

            // B button: Shoot mode control (shoot positions)
            if (gamepad1.b && !bPressed) {
                bPressed = true;
                shootMode = (shootMode + 1) % 4;  // Cycle through 0 -> 1 -> 2 -> 3 -> 0

                switch (shootMode) {
                    case 0:
                        axon.setPosition(0.0);  // Initial position
                        break;
                    case 1:
                        axon.setPosition(0.55); // Shoot first ball
                        break;
                    case 2:
                        axon.setPosition(0.9);  // Shoot second ball
                        break;
                    case 3:
                        axon.setPosition(0.15); // Shoot third ball
                        break;
                }
            }
            if (!gamepad1.b) bPressed = false;

            // Original level control (moved to different button if needed, or remove if not needed)
            if (gamepad1.left_stick_button && !pressed) {
                pressed = true;
                level = (level + 1) % 2;
            }
            if (!gamepad1.left_stick_button) pressed = false;

            speed = (level == 0) ? 0.8 : 0.4;

            // X button: Initialize/start outtake motor at current speed
            if (gamepad1.x && !xPressed) {
                xPressed = true;
                outtakePower = outtakeSpeed;
            }
            if (!gamepad1.x) xPressed = false;

            // Y button: Stop outtake motor
            if (gamepad1.y && !yPressed) {
                yPressed = true;
                outtakePower = 0;
            }
            if (!gamepad1.y) yPressed = false;

            // Dpad: Control outtake speed with 250ms delay between changes
            long currentTime = System.currentTimeMillis();
            if (currentTime - lastSpeedChangeTime >= 250) {
                if (gamepad1.dpad_up) {
                    outtakeSpeed = Math.min(1.0, outtakeSpeed + 0.05);  // Increase speed by 0.05
                    lastSpeedChangeTime = currentTime;
                }
                if (gamepad1.dpad_down) {
                    outtakeSpeed = Math.max(0.0, outtakeSpeed - 0.05);  // Decrease speed by 0.05
                    lastSpeedChangeTime = currentTime;
                }
            }

            // Real-time update: Always update outtake power if motor is running
            if (outtakePower > 0) {
                outtakePower = outtakeSpeed;
            }

            if (gamepad1.left_bumper) {
                myServo.setPosition(0.85);
                sleep(750);
                myServo.setPosition(0.25);
            }

            if (gamepad1.left_trigger > 0 && pos <= 1) {
                pos = pos + gamepad1.left_trigger;
            }

            if (gamepad1.right_trigger > 0 && pos >= 0) {
                pos = pos - gamepad1.right_trigger;
            }

            if (gamepad1.right_bumper) {
                power = -0.75;
            }
            else {
                power = 0;
            }

            double drive  =  -gamepad2.left_stick_y;
            double strafe = gamepad2.left_stick_x;
            double rotate =  -gamepad2.right_stick_x;

            drive  = Math.abs(drive)  > 0.1 ? drive  : 0;
            strafe = Math.abs(strafe) > 0.1 ? strafe : 0;
            rotate = Math.abs(rotate) > 0.1 ? rotate : 0;

            double flPower = speed * (rotate - strafe - drive);
            double frPower = speed * (rotate - strafe + drive);
            double blPower = speed * (rotate + strafe - drive);
            double brPower = speed * (rotate + strafe + drive);

            // normalize
            double max = Math.max(1.0,
                    Math.max(Math.abs(flPower), Math.max(Math.abs(frPower),
                            Math.max(Math.abs(blPower), Math.abs(brPower)))));
            flPower /= max; frPower /= max; blPower /= max; brPower /= max;

            frontLeft.setPower(flPower);
            frontRight.setPower(frPower);
            backLeft.setPower(blPower);
            backRight.setPower(brPower);
            Control.setPower(outtakePower);  // Use outtakePower controlled by X/Y buttons
            intake.setPower(power);

            telemetry.addData("Servo Position", pos);
            telemetry.addData("Axon Position", axon.getPosition());
            telemetry.addData("Input Mode", inputMode);
            telemetry.addData("Shoot Mode", shootMode);
            telemetry.addData("Outtake Power", outtakePower);
            telemetry.addData("Outtake Speed", outtakeSpeed);
            telemetry.addData("Speed", speed);
            telemetry.update();
        }
    }
}
