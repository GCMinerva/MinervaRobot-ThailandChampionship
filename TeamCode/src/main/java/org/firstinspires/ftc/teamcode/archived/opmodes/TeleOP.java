package org.firstinspires.ftc.teamcode.archived.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp(name="TeleOPPP")
public class TeleOP extends LinearOpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private Servo servo1;

    double speed = -0.8;
    double level = 0;
    double speed1 = 1;
    double level1 = 0;

    @Override
    public void runOpMode() {

        frontLeft  = hardwareMap.get(DcMotor.class, "fl");
        frontRight = hardwareMap.get(DcMotor.class, "fr");
        backLeft   = hardwareMap.get(DcMotor.class, "bl");
        backRight  = hardwareMap.get(DcMotor.class, "br");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        // Optional safety
        for (DcMotor m : new DcMotor[]{frontLeft, frontRight, backLeft, backRight}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        waitForStart();

        boolean pressed = false;

        while (opModeIsActive()) {

            if (gamepad1.a && !pressed) {
                pressed = true;
                level = (level + 1) % 2;
            }
            if (!gamepad1.a) pressed = false;

            speed = (level == 0) ? 0.8 : 0.4;

            double drive  =  -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double rotate =  -gamepad1.right_stick_x;

            drive  = Math.abs(drive)  > 0.1 ? drive  : 0;
            strafe = Math.abs(strafe) > 0.1 ? strafe : 0;
            rotate = Math.abs(rotate) > 0.1 ? rotate : 0;

            double flPower = speed * (rotate - strafe + drive);
            double frPower = speed * (rotate - strafe - drive);
            double blPower = speed * (rotate + strafe + drive);
            double brPower = speed * (rotate + strafe - drive);

            // normalize
            double max = Math.max(1.0,
                    Math.max(Math.abs(flPower), Math.max(Math.abs(frPower),
                            Math.max(Math.abs(blPower), Math.abs(brPower)))));
            flPower /= max; frPower /= max; blPower /= max; brPower /= max;

            frontLeft.setPower(flPower);
            frontRight.setPower(frPower);
            backLeft.setPower(blPower);
            backRight.setPower(brPower);
        }
    }
}
