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

    private final double MOTOR_SPEED = 0.5;

    @Override
    public void init() {
        outtake1 = hardwareMap.get(DcMotor.class, "outtake1");
        outtake2 = hardwareMap.get(DcMotor.class, "outtake2");

        outtake1.setDirection(DcMotor.Direction.FORWARD);
        outtake2.setDirection(DcMotor.Direction.FORWARD);
        outtake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Controls", "A: Toggle Motor 1, B: Toggle Motor 2");
        telemetry.update();
    }

    @Override
    public void loop() {
        if (gamepad1.a && !aPressed) {
            aPressed = true;
            motor1Power = (motor1Power == 0) ? MOTOR_SPEED : 0;
        }
        if (!gamepad1.a) {
            aPressed = false;
        }

        if (gamepad1.b && !bPressed) {
            bPressed = true;
            motor2Power = (motor2Power == 0) ? MOTOR_SPEED : 0;
        }
        if (!gamepad1.b) {
            bPressed = false;
        }

        outtake1.setPower(motor1Power);
        outtake2.setPower(motor2Power);

        telemetry.addData("=== TURRET TEST ===", "");
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
