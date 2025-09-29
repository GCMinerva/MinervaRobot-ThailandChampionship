package org.firstinspires.ftc.teamcode.config.hardware;

import com.qualcomm.robotcore.hardware.DcMotorImplEx;

public class CachedMotor {
    private double lastPower = 0;
    private final DcMotorImplEx motor;
    private double powerThreshold = 0.01;

    public CachedMotor(DcMotorImplEx motor, double powerThreshold) {
        this.motor = motor;
        this.powerThreshold = powerThreshold;
    }

    public CachedMotor(DcMotorImplEx motor) {
        this.motor = motor;
    }

    public void setPower(double power) {
        if ((Math.abs(this.lastPower - power) > this.powerThreshold) || (power == 0 && lastPower != 0)) {
            lastPower = power;
            motor.setPower(power);
        }
    }

    public int getPosition() {
        return (motor.getCurrentPosition());
    }

    public void setDirection(DcMotorImplEx.Direction direction) {
        this.motor.setDirection(direction);
    }

    public void setCachingThreshold(double powerThreshold) {
        this.powerThreshold = powerThreshold;
    }

    public double getPower() {
        return lastPower;
    }

    public void setMode(DcMotorImplEx.RunMode runMode) {
        this.motor.setMode(runMode);
    }

    public void setZeroPowerBehavior(DcMotorImplEx.ZeroPowerBehavior zeroPowerBehavior) {
        this.motor.setZeroPowerBehavior(zeroPowerBehavior);
    }
}