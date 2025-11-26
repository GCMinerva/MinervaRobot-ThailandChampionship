package org.firstinspires.ftc.teamcode.archived.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="hello")
public class Control extends LinearOpMode {

    private DcMotor Control;
    private Servo myServo;

    double speed = -0.8;
    double level = 0;
    double speed1 = 1;
    double level1 = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        // Map motors
        Control = hardwareMap.get(DcMotor.class, "m1");
        // Map servo
        myServo = hardwareMap.get(Servo.class, "servo");
        myServo.setPosition(0.45);
        waitForStart();
        // ถือว่าเป็น 0 เริ่มต้น
        while (opModeIsActive()) {

            // ควบคุมมอเตอร์
            if (gamepad1.a) {
                speed = speed - 0.01;
                sleep(300);
            }
            if (gamepad1.b) {
                speed = speed + 0.01;
                sleep(300);
            }
            Control.setPower(speed);


            if (gamepad1.y) {
                myServo.setPosition(0);
                sleep(500);
                myServo.setPosition(0.45);
            }

            telemetry.addData("Motor Speed", speed);
            telemetry.addData("Servo Position", myServo.getPosition());
            telemetry.update();
        }
    }
}
