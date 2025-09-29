package org.firstinspires.ftc.teamcode.config.core.subsystems;

import static org.firstinspires.ftc.teamcode.config.core.RobotConstants.liftToFinish;
import static org.firstinspires.ftc.teamcode.config.core.RobotConstants.liftToHang;
import static org.firstinspires.ftc.teamcode.config.core.RobotConstants.liftToHighBasket;
import static org.firstinspires.ftc.teamcode.config.core.RobotConstants.liftToIdle;
import static org.firstinspires.ftc.teamcode.config.core.RobotConstants.liftToLowBasket;
import static org.firstinspires.ftc.teamcode.config.core.RobotConstants.liftToPickup;
import static org.firstinspires.ftc.teamcode.config.core.RobotConstants.liftToSpecimen;
import static org.firstinspires.ftc.teamcode.config.core.RobotConstants.liftToTransfer;
import static org.firstinspires.ftc.teamcode.config.core.RobotConstants.liftToZero;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.config.hardware.CachedMotor;
import org.firstinspires.ftc.teamcode.config.hardware.Encoder;

@Config
public class Lift {

    private MultipleTelemetry telemetry;

    public CachedMotor rightLift, leftLift;
    public Encoder leftLiftEncoder;
    public int pos;
    public PIDController pid;
    public int pidLevel = 0;
    public static int target;
    public static double p = 0.01, i = 0, d = 0.000005, f = 0.05;

    public Lift(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        rightLift = new CachedMotor(hardwareMap.get(DcMotorImplEx.class, "lR"), 0.01);
        leftLift = new CachedMotor(hardwareMap.get(DcMotorImplEx.class, "lL"), 0.01);

        rightLift.setDirection(DcMotorImplEx.Direction.REVERSE);
        leftLift.setDirection(DcMotorImplEx.Direction.FORWARD);

        leftLiftEncoder = new Encoder(hardwareMap.get(DcMotorImplEx.class, "lL"));
        leftLiftEncoder.setMultiplier(Encoder.FORWARD);

//        leftLift.setMode(DcMotorImplEx.RunMode.STOP_AND_RESET_ENCODER);

        rightLift.setMode(DcMotorImplEx.RunMode.RUN_WITHOUT_ENCODER);
        leftLift.setMode(DcMotorImplEx.RunMode.RUN_WITHOUT_ENCODER);

        pid = new PIDController(p, i, d);
    }

    public void update() {
        if (pidLevel == 1) {
            double pid_output = pid.calculate(getPos(), target);
            double power = pid_output + f;

            if (getPos() < 50 && target < 50) {
                rightLift.setPower(0);
                leftLift.setPower(0);
            } else {
                rightLift.setPower(power);
                leftLift.setPower(power);
            }
        } else if (pidLevel == 2) {
            target = getPos();
        }
    }

    public void manual(double left, double right) {
        if (Math.abs(left) > 0.05 || Math.abs(right) > 0.05) {
            pidLevel = 2;
            leftLift.setPower(right - left);
            rightLift.setPower(right - left);
        } else if (pidLevel == 2) {
//            pidLevel = 1;
            rightLift.setPower(0);
            leftLift.setPower(0);
        }
    }

    public void setTarget(int b) {
        pidLevel = 1;
        target = b;
    }

    public void setPidLevel(int x) {
        pidLevel = x;
        if (x == 1) {
            pid.setPID(p, i, d);
            rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        } else if (x == 2) {
            rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    public int getPos() {
        pos = (int) leftLiftEncoder.getPosition();
        return pos;
    }

    public int getTarget() {
        return target;
    }

    public void init() {
        pid.setPID(p, i, d);
    }

    public void start() {
        toTransfer();
    }

    public void toZero() {
        setTarget(liftToZero);
    }

    public void toTransfer() {
        setTarget(liftToTransfer);
    }

    public void toPickup() {
        setTarget(liftToPickup);
    }

    public void toSpecimen() {
        setTarget(liftToSpecimen);
    }

    public void toHighBasket() {
        setTarget(liftToHighBasket);
    }

    public void toLowBasket() {
        setTarget(liftToLowBasket);
    }

    public void toHang() {
        setTarget(liftToHang);
    }

    public void toFinish() {
        setTarget(liftToFinish);
    }

    public void toIdle() {
        setTarget(liftToIdle);
    }

    public boolean roughlyAtTarget() {
        return Math.abs(getPos() - target) < 50;
    }

    public boolean halfwayToTarget() { return Math.abs(getPos() - target) < target / 2; }

    public boolean percentageToTarget(double p) {
        return Math.abs(getPos() - target) <= target * p;
    }

    public void pidOn() {
        pidLevel = 1;
    }

    public void pidOff() {
        pidLevel = 2;
    }

    public void telemetry() {
        telemetry.addData("Lift Pos: ", getPos());
        telemetry.addData("Lift Target: ", target);
        telemetry.addData("Left Power: ", leftLift.getPower());
    }

    public void periodic() {
        update();
        telemetry();
    }
}
