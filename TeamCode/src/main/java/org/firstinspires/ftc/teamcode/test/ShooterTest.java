package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Outtake Test", group = "Test")
public class ShooterTest extends OpMode {

    private DcMotor outtake1;
    private DcMotor outtake2;

    @Override
    public void init() {
        // Initialize outtake motors
        outtake1 = hardwareMap.get(DcMotor.class, "outtake1");
        outtake2 = hardwareMap.get(DcMotor.class, "outtake2");

        // Match ActuatorSubsystem configuration
        outtake1.setDirection(DcMotor.Direction.REVERSE);
        outtake2.setDirection(DcMotor.Direction.REVERSE);

        outtake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        outtake1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        outtake2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Controls", "Left Stick Y = Outtake Power");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Get joystick input (negative because Y is inverted)
        double outtakePower = -gamepad1.left_stick_y;

        // Set both outtake motors to same power
        outtake1.setPower(outtakePower);
        outtake2.setPower(outtakePower);

        // Display telemetry
        telemetry.addData("Outtake Power", "%.2f", outtakePower);
        telemetry.addData("Left Stick Y", "%.2f", gamepad1.left_stick_y);
        telemetry.update();
    }

    @Override
    public void stop() {
        // Stop outtake motors when OpMode ends
        outtake1.setPower(0);
        outtake2.setPower(0);
    }
}
