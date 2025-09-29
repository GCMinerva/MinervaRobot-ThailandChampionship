package org.firstinspires.ftc.teamcode.config.core.subsystems;

import static org.firstinspires.ftc.teamcode.config.core.RobotConstants.extendAutoFull;
import static org.firstinspires.ftc.teamcode.config.core.RobotConstants.extendFirst;
import static org.firstinspires.ftc.teamcode.config.core.RobotConstants.extendFull;
import static org.firstinspires.ftc.teamcode.config.core.RobotConstants.extendGrab;
import static org.firstinspires.ftc.teamcode.config.core.RobotConstants.extendHoldback;
import static org.firstinspires.ftc.teamcode.config.core.RobotConstants.extendInches;
import static org.firstinspires.ftc.teamcode.config.core.RobotConstants.extendSecond;
import static org.firstinspires.ftc.teamcode.config.core.RobotConstants.extendZero;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Extend {

    public enum ExtendState {
        ZERO, FULL, GRAB, HOLDBACK, FIRST, SECOND, INCHES, AUTOFULL
    }

    private MultipleTelemetry telemetry;

    public ServoImplEx left, right;
    private ExtendState state = ExtendState.ZERO;
    private double pos = 0;

    public Extend(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        left = hardwareMap.get(ServoImplEx.class, "eL");
        right = hardwareMap.get(ServoImplEx.class, "eR");

        left.setDirection(ServoImplEx.Direction.FORWARD);
        right.setDirection(ServoImplEx.Direction.REVERSE);
    }

    public void setTarget(double b) {
        left.setPosition(b);
        right.setPosition(b);
        pos = b;
    }

    public void toZero() {
        setTarget(extendZero);
        state = ExtendState.ZERO;
    }

    public void toFull() {
        setTarget(extendFull);
        state = ExtendState.FULL;
    }

    public void toAutoFull() {
        setTarget(extendAutoFull);
        state = ExtendState.AUTOFULL;
    }

    public void toGrab() {
        setTarget(extendGrab);
        state = ExtendState.GRAB;
    }

    public void toHoldback() {
        setTarget(extendHoldback);
        state = ExtendState.HOLDBACK;
    }
    public void toFirst() {
        setTarget(extendFirst);
        state = ExtendState.FIRST;
    }
    public void toSecond() {
        setTarget(extendSecond);
        state = ExtendState.SECOND;
    }

    public void toInches(double inches, double deltaX) {
        setTarget(extendInches(inches+deltaX));
        state = ExtendState.INCHES;
    }

//    public void extendInches(double inches) {
//        double temp = extendZero - (inches * extendInchesPer);
//
//        if(temp < extendFull) {
//            temp = extendFull;
//        }
//
//        setTarget(temp);
//    }

    public double getPos() {
        pos = right.getPosition();
        return pos;
    }

    public ExtendState getState() {
        return state;
    }

    public void switchExtendState() {
        if (state == ExtendState.FULL) {
            toZero();
        } else {
            toFull();
        }
    }

    public void init() {
        toZero();
    }

    public void start() {
        toZero();
    }

    public double extendInches(double inches) {
        if (inches == 20) return extendInches[20];
        int idx = (int)Math.floor(inches);
        double l = extendInches[idx];
        double r = extendInches[idx+1];
        return l+(r-l)*(inches-idx);
    }

    public void telemetry() {
        telemetry.addData("Extend Pos: ", getPos());
    }

    public void periodic() {
        telemetry();
    }
}