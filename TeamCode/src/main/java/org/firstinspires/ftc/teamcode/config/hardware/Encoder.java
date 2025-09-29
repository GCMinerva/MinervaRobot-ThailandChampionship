package org.firstinspires.ftc.teamcode.config.hardware;

import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;

public class Encoder {
    private DcMotorImplEx motor;
    private double previousPosition;
    private double currentPosition;
    private double multiplier;
    public static final double FORWARD = (double) 1.0F;
    public static final double REVERSE = (double) -1.0F;

    public Encoder(DcMotorImplEx setMotor) {
        this.motor = setMotor;
        this.multiplier = (double) 1.0F;
//        this.reset();
    }

    public void setMultiplier(double multiplier) {
        this.multiplier = multiplier;
    }

    public void reset() {
        this.motor.setMode(RunMode.STOP_AND_RESET_ENCODER);
        this.previousPosition = (double) this.motor.getCurrentPosition();
        this.currentPosition = (double) this.motor.getCurrentPosition();
        this.motor.setMode(RunMode.RUN_WITHOUT_ENCODER);
    }

    public void update() {
        this.previousPosition = this.currentPosition;
        this.currentPosition = (double) this.motor.getCurrentPosition();
    }

    public double getMultiplier() {
        return this.multiplier * (double) (this.motor.getDirection() == Direction.FORWARD ? 1 : -1);
    }

    public double getDeltaPosition() {
        return this.getMultiplier() * (this.currentPosition - this.previousPosition);
    }

    public double getPosition() {
        return this.getMultiplier() * this.motor.getCurrentPosition();
    }
}
