package org.firstinspires.ftc.teamcode.archived.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="tele")
public class Tele extends LinearOpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    double speed = 1;
    double level = 0;
    double speed1 = 1;
    double level1 = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        // Robot robot = new Robot(hardwareMap, telemetry, new GamepadTracker(gamepad1), new GamepadTracker(gamepad2), true);

        // Map motors
        frontLeft = hardwareMap.get(DcMotor.class, "fl");
        frontRight = hardwareMap.get(DcMotor.class, "fr");
        backLeft = hardwareMap.get(DcMotor.class, "bl");
        backRight = hardwareMap.get(DcMotor.class, "br");
//        sh = hardwareMap.get(DcMotor.class, "shoot");
        frontRight.setDirection(DcMotor.Direction.REVERSE);
//        backRight.setDirection(DcMotor.Direction.REVERSE);
        waitForStart();

        boolean pressed = false;
//        boolean spressed = false;

        while (opModeIsActive()) {

            if (gamepad1.a && !pressed) {
                pressed = true;
                level = (level + 1) % 2;
            }
            if (!gamepad1.a) {
                pressed = false;
            }

//            if (gamepad1.left_bumper && !spressed) {
//                spressed = true;
//                level1 = (level1 + 1) % 2;
//            }
//            if (!gamepad1.left_bumper) {
//                spressed = false;
//            }

            speed = (level == 0) ? 1 : 0.5;
//            speed1 = (level1 == 0) ? 1 : 0.5;/

            double drive = -gamepad1.left_stick_y;
            double strafe = -gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;
//            double shoot = gamepad1.left_trigger;

            drive = Math.abs(drive) > 0.1 ? drive : 0;
            strafe = Math.abs(strafe) > 0.1 ? strafe : 0;
            rotate = Math.abs(rotate) > 0.1 ? rotate : 0;

            double flPower = speed * (rotate - strafe + drive);
            double frPower = speed * (rotate + strafe - drive);
            double blPower = speed * (rotate + strafe + drive);
            double brPower = speed * (rotate - strafe - drive);
//            double shPower = speed1 * shoot;

            frontLeft.setPower(flPower);
            frontRight.setPower(frPower);
            backLeft.setPower(blPower);
            backRight.setPower(brPower);
//            sh.setPower(shPower);
        }
    }
}
