package org.firstinspires.ftc.teamcode.config.core.subsystems;

import static org.firstinspires.ftc.teamcode.config.core.RobotConstants.outtakeGrabClosed;
import static org.firstinspires.ftc.teamcode.config.core.RobotConstants.outtakeGrabLoose;
import static org.firstinspires.ftc.teamcode.config.core.RobotConstants.outtakeGrabOpen;
import static org.firstinspires.ftc.teamcode.config.core.RobotConstants.outtakeGrabOpenFull;
import static org.firstinspires.ftc.teamcode.config.core.RobotConstants.outtakePivotLeftOffset;
import static org.firstinspires.ftc.teamcode.config.core.RobotConstants.outtakePivotPickup;
import static org.firstinspires.ftc.teamcode.config.core.RobotConstants.outtakePivotScore;
import static org.firstinspires.ftc.teamcode.config.core.RobotConstants.outtakePivotSpecimen;
import static org.firstinspires.ftc.teamcode.config.core.RobotConstants.outtakePivotTransfer;
import static org.firstinspires.ftc.teamcode.config.core.RobotConstants.outtakeRotatePickup;
import static org.firstinspires.ftc.teamcode.config.core.RobotConstants.outtakeRotateScore;
import static org.firstinspires.ftc.teamcode.config.core.RobotConstants.outtakeRotateSpecimen;
import static org.firstinspires.ftc.teamcode.config.core.RobotConstants.outtakeRotateTransfer;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Outtake {

    public enum GrabState {
        CLOSED, OPEN, OPENFULL, LOOSE
    }

    public enum RotateState {
        TRANSFER, SCORE, PICKUP, SPECIMEN
    }

    public enum PivotState {
        TRANSFER, SCORE, PICKUP, SPECIMEN
    }

    public ServoImplEx grab, rotate, leftPivot, rightPivot;
    public GrabState grabState;
    public RotateState rotateState;
    public PivotState pivotState;
    private Telemetry telemetry;
    public Timer grabTimer;

    public Outtake(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        grab = hardwareMap.get(ServoImplEx.class, "oG");
        rotate = hardwareMap.get(ServoImplEx.class, "oR");
        leftPivot = hardwareMap.get(ServoImplEx.class, "oLP");
        rightPivot = hardwareMap.get(ServoImplEx.class, "oRP");

        leftPivot.setDirection(ServoImplEx.Direction.REVERSE);
        rightPivot.setDirection(ServoImplEx.Direction.FORWARD);

        rotate.setDirection(ServoImplEx.Direction.REVERSE);

        grabTimer = new Timer();
    }

    public void setRotateState(RotateState state) {
        if (state == RotateState.TRANSFER) {
            rotate.setPosition(outtakeRotateTransfer);
            this.rotateState = RotateState.TRANSFER;
        } else if (state == RotateState.SCORE) {
            rotate.setPosition(outtakeRotateScore);
            this.rotateState = RotateState.SCORE;
        } else if (state == RotateState.PICKUP) {
            rotate.setPosition(outtakeRotatePickup);
            this.rotateState = RotateState.PICKUP;
        } else if (state == RotateState.SPECIMEN) {
            rotate.setPosition(outtakeRotateSpecimen);
            this.rotateState = RotateState.SPECIMEN;
        }
    }

    public void setPivotState(PivotState pivotState) {
        if (pivotState == PivotState.TRANSFER) {
            leftPivot.setPosition(outtakePivotTransfer + outtakePivotLeftOffset);
            rightPivot.setPosition(outtakePivotTransfer);
            this.pivotState = PivotState.TRANSFER;
        } else if (pivotState == PivotState.SCORE) {
            leftPivot.setPosition(outtakePivotScore + outtakePivotLeftOffset);
            rightPivot.setPosition(outtakePivotScore);
            this.pivotState = PivotState.SCORE;
        } else if (pivotState == PivotState.PICKUP) {
            leftPivot.setPosition(outtakePivotPickup + outtakePivotLeftOffset);
            rightPivot.setPosition(outtakePivotPickup);
            this.pivotState = PivotState.PICKUP;
        } else if (pivotState == PivotState.SPECIMEN) {
            leftPivot.setPosition(outtakePivotSpecimen + outtakePivotLeftOffset);
            rightPivot.setPosition(outtakePivotSpecimen);
            this.pivotState = PivotState.SPECIMEN;
        }
    }

    public void setGrabState(GrabState grabState) {
        if (grabState == GrabState.CLOSED) {
            grab.setPosition(outtakeGrabClosed);
            this.grabState = GrabState.CLOSED;
        } else if (grabState == GrabState.OPEN) {
            grab.setPosition(outtakeGrabOpen);
            this.grabState = GrabState.OPEN;
        } else if (grabState == GrabState.OPENFULL) {
            grab.setPosition(outtakeGrabOpenFull);
            this.grabState = GrabState.OPENFULL;
        } else if (grabState == GrabState.LOOSE) {
            grab.setPosition(outtakeGrabLoose);
            this.grabState = GrabState.LOOSE;
        }
    }

    public void switchGrabState() {
        if (grabState == GrabState.CLOSED || grabState == GrabState.LOOSE) {
            setGrabState(GrabState.OPENFULL);
        } else {
            setGrabState(GrabState.CLOSED);
        }
    }

    private double grabwait = 0;
    public void openfull() {
        setGrabState(GrabState.OPENFULL);
    }
    public void open() {
        setGrabState(GrabState.OPEN);
    }
    public void loose() {
        setGrabState(GrabState.LOOSE);
    }

    public void close() {
        setGrabState(GrabState.CLOSED);
    }

    public void transfer() {
        grabwait = 0.25;
        setRotateState(RotateState.TRANSFER);
        setPivotState(PivotState.TRANSFER);
        setGrabState(GrabState.OPEN);
        grabTimer.resetTimer();
    }

    public void score() {
        setRotateState(RotateState.SCORE);
        setPivotState(PivotState.SCORE);
        setGrabState(GrabState.LOOSE);
    }

    public void pickup() {
        grabwait = (outtakePivotPickup-rightPivot.getPosition())/(outtakePivotPickup-outtakePivotScore)*0.8+0.25;
        setRotateState(RotateState.PICKUP);
        setPivotState(PivotState.PICKUP);
        setGrabState(GrabState.OPEN);
        grabTimer.resetTimer();
    }

    public void pickup2() {
        grabwait = (outtakePivotPickup-rightPivot.getPosition())/(outtakePivotPickup-outtakePivotScore)*0.8+0.25;
        setRotateState(RotateState.PICKUP);
        setPivotState(PivotState.PICKUP);
        setGrabState(GrabState.OPEN);
        grabTimer.resetTimer();
    }

    public void specimen() {
        setRotateState(RotateState.SPECIMEN);
        setPivotState(PivotState.SPECIMEN);
        setGrabState(GrabState.CLOSED);
    }

    public void setRotate(double b) {
        rotate.setPosition(b);
    }

    public void setPivot(double b) {
        leftPivot.setPosition(b + outtakePivotLeftOffset);
        rightPivot.setPosition(b);
    }

    public void setGrab(double b) {
        grab.setPosition(b);
    }

    public void init() {
//        setPivotState(PivotState.PICKUP);
//        setRotateState(RotateState.PICKUP);
        setPivotState(PivotState.SPECIMEN);
        setRotateState(RotateState.SPECIMEN);
        setGrabState(GrabState.CLOSED);
    }

    public void start() {
        setPivotState(PivotState.PICKUP);
        setRotateState(RotateState.PICKUP);
        setGrabState(GrabState.OPEN);
        grabwait = 1;
        grabTimer.resetTimer();
    }

    public void telemetry() {
        telemetry.addData("Outtake Grab State: ", grabState);
        telemetry.addData("Outtake Grab Pos: ", grab.getPosition());
        telemetry.addData("Outtake Rotate State: ", rotateState);
        telemetry.addData("Outtake Pivot Pos: ", rotate.getPosition());
        telemetry.addData("Outtake Pivot State: ", pivotState);
        telemetry.addData("Outtake Pivot Pos: ", leftPivot.getPosition());
    }

    public void periodic() {
        telemetry();
        if ((this.pivotState == PivotState.PICKUP || this.pivotState == PivotState.TRANSFER)
            && grabState == GrabState.OPEN && grabTimer.getElapsedTimeSeconds() > grabwait) {
            setGrabState(GrabState.OPENFULL);
        }
    }
}
