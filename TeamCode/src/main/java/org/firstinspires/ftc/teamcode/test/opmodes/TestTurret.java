package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="TestTurret")
public class TestTurret extends OpMode {

    private DcMotor outtake1, outtake2;

    private boolean aPressed = false;
    private boolean bPressed = false;

    private double motor1Power = 0;
    private double motor2Power = 0;

    private double motorSpeed = 0.5;
    private long lastSpeedChangeTime = 0;

    @Override
    public void init() {
        outtake1 = hardwareMap.get(DcMotor.class, "outtake1");
        outtake2 = hardwareMap.get(DcMotor.class, "outtake2");

        outtake1.setDirection(DcMotor.Direction.REVERSE);
        outtake2.setDirection(DcMotor.Direction.REVERSE);
        outtake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Controls", "A: Motor 1 | B: Motor 2");
        telemetry.addData("Controls", "DPAD UP/DOWN: Speed");
        telemetry.update();
    }

    @Override
    public void loop() {
        if (gamepad1.a && !aPressed) {
            aPressed = true;
            motor1Power = (motor1Power == 0) ? motorSpeed : 0;
        }
        if (!gamepad1.a) {
            aPressed = false;
        }

        if (gamepad1.b && !bPressed) {
            bPressed = true;
            motor2Power = (motor2Power == 0) ? motorSpeed : 0;
        }
        if (!gamepad1.b) {
            bPressed = false;
        }

        long currentTime = System.currentTimeMillis();
        if (currentTime - lastSpeedChangeTime >= 250) {
            if (gamepad1.dpad_up) {
                motorSpeed = Math.min(1.0, motorSpeed + 0.05);
                lastSpeedChangeTime = currentTime;
            }
            if (gamepad1.dpad_down) {
                motorSpeed = Math.max(0.0, motorSpeed - 0.05);
                lastSpeedChangeTime = currentTime;
            }
        }

        if (motor1Power > 0) {
            motor1Power = motorSpeed;
        }
        if (motor2Power > 0) {
            motor2Power = motorSpeed;
        }

        outtake1.setPower(motor1Power);
        outtake2.setPower(motor2Power);

        telemetry.addData("=== TURRET TEST ===", "");
        telemetry.addData("Motor Speed Setting", "%.2f", motorSpeed);
        telemetry.addData("", "");
        telemetry.addData("Motor 1 (A)", motor1Power > 0 ? "RUNNING" : "STOPPED");
        telemetry.addData("Motor 1 Power", "%.2f", motor1Power);
        telemetry.addData("", "");
        telemetry.addData("Motor 2 (B)", motor2Power > 0 ? "RUNNING" : "STOPPED");
        telemetry.addData("Motor 2 Power", "%.2f", motor2Power);
        telemetry.update();
    }

    @Override
    public void stop() {
        outtake1.setPower(0);
        outtake2.setPower(0);
    }
}
