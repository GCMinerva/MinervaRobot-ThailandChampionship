package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.PoseTracker;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Vector;

@TeleOp(name = "GetPosition", group = "Linear Opmode")
public class GetPosition extends LinearOpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private PoseTracker poseTracker;
    private FourWheelLocalizer localizer;

    // Localizer minimal สำหรับ mecanum 4 ล้อ
    class FourWheelLocalizer implements Localizer {
        private DcMotor fl, fr, bl, br;
        private Pose currentPose;

        public FourWheelLocalizer(DcMotor fl, DcMotor fr, DcMotor bl, DcMotor br) {
            this.fl = fl;
            this.fr = fr;
            this.bl = bl;
            this.br = br;
            this.currentPose = new Pose(0,0,0);
        }

        @Override
        public Pose getPose() {
            return currentPose;
        }

        @Override
        public Pose getVelocity() {
            return new Pose(0,0,0);
        }

        @Override
        public Vector getVelocityVector() {
            return new Vector(0,0);
        }

        @Override
        public void setStartPose(Pose setStart) { }

        @Override
        public void setPose(Pose setPose) {
            this.currentPose = setPose;
        }

        @Override
        public void update() {
            // อ่าน encoder ของล้อแล้วประมาณ x,y,heading
            int flTicks = fl.getCurrentPosition();
            int frTicks = fr.getCurrentPosition();
            int blTicks = bl.getCurrentPosition();
            int brTicks = br.getCurrentPosition();

            // ใส่ formula คำนวณ mecanum movement ที่นี่
            // สำหรับ skeleton ให้ยังไม่ทำ
        }

        @Override
        public double getTotalHeading() { return 0; }

        @Override
        public double getForwardMultiplier() { return 1; }

        @Override
        public double getLateralMultiplier() { return 1; }

        @Override
        public double getTurningMultiplier() { return 1; }

        @Override
        public void resetIMU() throws InterruptedException { }

        @Override
        public double getIMUHeading() { return 0; }

        @Override
        public boolean isNAN() { return false; }
    }


    @Override
    public void runOpMode() {

        // map มอเตอร์
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");

        // ตั้งทิศทาง
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        // สร้าง localizer
        localizer = new FourWheelLocalizer(frontLeft, frontRight, backLeft, backRight);

        // ส่งเข้า PoseTracker
        poseTracker = new PoseTracker(localizer);

        // ตั้งค่า starting pose
        poseTracker.setStartingPose(new Pose(0, 0, 0));

        waitForStart();

        while (opModeIsActive()) {
            // รับค่าจากจอย
            double y = -gamepad1.left_stick_y;   // เดินหน้า/ถอยหลัง
            double x = gamepad1.left_stick_x;    // slide ซ้าย/ขวา
            double rx = gamepad1.right_stick_x;  // หมุน

            // คำนวณ power ของ mecanum
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double flPower = (y + x + rx) / denominator;
            double blPower = (y - x + rx) / denominator;
            double frPower = (y - x - rx) / denominator;
            double brPower = (y + x - rx) / denominator;

            // ส่ง power ไปมอเตอร์
            frontLeft.setPower(flPower);
            backLeft.setPower(blPower);
            frontRight.setPower(frPower);
            backRight.setPower(brPower);

            // อัพเดตตำแหน่งผ่าน PoseTracker
            poseTracker.update();

            // แสดงค่า Pose
            Pose pose = poseTracker.getPose();
            telemetry.addData("X", pose.getX());
            telemetry.addData("Y", pose.getY());
            telemetry.addData("Heading (rad)", pose.getHeading());
            telemetry.update();
        }
    }
}