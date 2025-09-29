package org.firstinspires.ftc.teamcode.config.core.subsystems;

import static org.firstinspires.ftc.teamcode.config.core.RobotConstants.extendFull;
import static org.firstinspires.ftc.teamcode.config.core.RobotConstants.extendZero;
import static org.firstinspires.ftc.teamcode.config.core.RobotConstants.intakeGrabClosed;
import static org.firstinspires.ftc.teamcode.config.core.RobotConstants.intakeGrabLoose;
import static org.firstinspires.ftc.teamcode.config.core.RobotConstants.intakeGrabOpen;
import static org.firstinspires.ftc.teamcode.config.core.RobotConstants.intakeGrabOpenFull;
import static org.firstinspires.ftc.teamcode.config.core.RobotConstants.intakePivotGround;
import static org.firstinspires.ftc.teamcode.config.core.RobotConstants.intakePivotGroundOutward;
import static org.firstinspires.ftc.teamcode.config.core.RobotConstants.intakePivotGroundOutwardFull;
import static org.firstinspires.ftc.teamcode.config.core.RobotConstants.intakePivotHoldback;
import static org.firstinspires.ftc.teamcode.config.core.RobotConstants.intakePivotHover;
import static org.firstinspires.ftc.teamcode.config.core.RobotConstants.intakePivotHoverOutward;
import static org.firstinspires.ftc.teamcode.config.core.RobotConstants.intakePivotHoverOutwardFull;
import static org.firstinspires.ftc.teamcode.config.core.RobotConstants.intakePivotThrowback;
import static org.firstinspires.ftc.teamcode.config.core.RobotConstants.intakePivotTransfer;
import static org.firstinspires.ftc.teamcode.config.core.RobotConstants.intakeRotateDodge;
import static org.firstinspires.ftc.teamcode.config.core.RobotConstants.intakeRotateGround;
import static org.firstinspires.ftc.teamcode.config.core.RobotConstants.intakeRotateGroundOutward;
import static org.firstinspires.ftc.teamcode.config.core.RobotConstants.intakeRotateGroundOutwardFull;
import static org.firstinspires.ftc.teamcode.config.core.RobotConstants.intakeRotateHoldback;
import static org.firstinspires.ftc.teamcode.config.core.RobotConstants.intakeRotateHover;
import static org.firstinspires.ftc.teamcode.config.core.RobotConstants.intakeRotateHoverOutward;
import static org.firstinspires.ftc.teamcode.config.core.RobotConstants.intakeRotateHoverOutwardFull;
import static org.firstinspires.ftc.teamcode.config.core.RobotConstants.intakeRotateLeftOffset;
import static org.firstinspires.ftc.teamcode.config.core.RobotConstants.intakeRotatePerDegree;
import static org.firstinspires.ftc.teamcode.config.core.RobotConstants.intakeRotateThrowback;
import static org.firstinspires.ftc.teamcode.config.core.RobotConstants.intakeRotateTransfer;
import static org.firstinspires.ftc.teamcode.config.core.RobotConstants.intakeRotateTransferWait;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake {

    public enum RotateState {
        HOVER, HOVERO, GROUND, GROUNDO, GROUNDOZ, TRANSFER, THROWBACK, HOLDBACK, DODGE, TRANSFERWAIT
    }

    public enum PivotState {
        HOVER, HOVERO, GROUND, GROUNDO, GROUNDOZ, TRANSFER, THROWBACK, HOLDBACK
    }

    public enum GrabState {
        CLOSED, OPEN, OPENFULL, LOOSE
    }

    public ServoImplEx grab, leftRotate, rightRotate, leftPivot, rightPivot;
    public RotateState rotateState;
    public PivotState pivotState;
    public GrabState grabState;
    private MultipleTelemetry telemetry;

    public Timer grabTimer, rotateTimer;
    public double rotateDegrees = 0;
    public double extendP = 0;

    public Intake(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        grab = hardwareMap.get(ServoImplEx.class, "iG");
        leftRotate = hardwareMap.get(ServoImplEx.class, "iLR");
        rightRotate = hardwareMap.get(ServoImplEx.class, "iRR");
//        leftPivot = hardwareMap.get(ServoImplEx.class, "iLP");
        rightPivot = hardwareMap.get(ServoImplEx.class, "iRP");

//        leftPivot.setDirection(ServoImplEx.Direction.REVERSE);
        rightPivot.setDirection(ServoImplEx.Direction.FORWARD);

        leftRotate.setDirection(ServoImplEx.Direction.FORWARD);
        rightRotate.setDirection(ServoImplEx.Direction.REVERSE);

        grabTimer = new Timer();
        rotateTimer = new Timer();
    }

    public void setGrabState(GrabState grabState) {
        if (grabState == GrabState.CLOSED) {
            grab.setPosition(intakeGrabClosed);
            this.grabState = GrabState.CLOSED;
        } else if (grabState == GrabState.OPEN) {
            grab.setPosition(intakeGrabOpen);
            this.grabState = GrabState.OPEN;
        } else if (grabState == GrabState.OPENFULL) {
            grab.setPosition(intakeGrabOpenFull);
            this.grabState = GrabState.OPENFULL;
        } else if (grabState == GrabState.LOOSE) {
            grab.setPosition(intakeGrabLoose);
            this.grabState = GrabState.LOOSE;
        }
    }

    public void setRotateState(RotateState state) {
        if (state == RotateState.HOVER) {
            leftRotate.setPosition(intakeRotateHover + rotateDegrees * intakeRotatePerDegree + intakeRotateLeftOffset);
            rightRotate.setPosition(intakeRotateHover - rotateDegrees * intakeRotatePerDegree);
            this.rotateState = RotateState.HOVER;
        } else if (state == RotateState.HOVERO) {
            double l = intakeRotateHoverOutward;
            double r = intakeRotateHoverOutwardFull;
            double x = extendP*(r-l)+l;
            leftRotate.setPosition(x + rotateDegrees * intakeRotatePerDegree + intakeRotateLeftOffset);
            rightRotate.setPosition(x - rotateDegrees * intakeRotatePerDegree);
            this.rotateState = RotateState.HOVERO;
        } else if (state == RotateState.GROUND) {
            leftRotate.setPosition(intakeRotateGround + rotateDegrees * intakeRotatePerDegree + intakeRotateLeftOffset);
            rightRotate.setPosition(intakeRotateGround - rotateDegrees * intakeRotatePerDegree);
            this.rotateState = RotateState.GROUND;
        } else if (state == RotateState.GROUNDO) {
            double l = intakeRotateGroundOutward;
            double r = intakeRotateGroundOutwardFull;
            double x = extendP*(r-l)+l;
            leftRotate.setPosition(x + rotateDegrees * intakeRotatePerDegree + intakeRotateLeftOffset);
            rightRotate.setPosition(x - rotateDegrees * intakeRotatePerDegree);
            this.rotateState = RotateState.GROUNDO;
        } else if (state == RotateState.GROUNDOZ) {
            double l = intakeRotateGroundOutward;
            double r = intakeRotateGroundOutwardFull;
            double x = extendP*(r-l)+l;
            leftRotate.setPosition(x-0.0225 + rotateDegrees * intakeRotatePerDegree + intakeRotateLeftOffset);
            rightRotate.setPosition(x-0.0225 - rotateDegrees * intakeRotatePerDegree);
            this.rotateState = RotateState.GROUNDOZ;
        } else if (state == RotateState.TRANSFER) {
            leftRotate.setPosition(intakeRotateTransfer + intakeRotateLeftOffset);
            rightRotate.setPosition(intakeRotateTransfer);
            this.rotateState = RotateState.TRANSFER;
        } else if (state == RotateState.THROWBACK) {
            leftRotate.setPosition(intakeRotateThrowback + intakeRotateLeftOffset);
            rightRotate.setPosition(intakeRotateThrowback);
            this.rotateState = RotateState.THROWBACK;
        } else if (state == RotateState.HOLDBACK) {
            leftRotate.setPosition(intakeRotateHoldback + intakeRotateLeftOffset);
            rightRotate.setPosition(intakeRotateHoldback);
            this.rotateState = RotateState.HOLDBACK;
        } else if (state == RotateState.DODGE) {
            leftRotate.setPosition(intakeRotateDodge + intakeRotateLeftOffset);
            rightRotate.setPosition(intakeRotateDodge);
            this.rotateState = RotateState.DODGE;
        } else if (state == RotateState.TRANSFERWAIT) {
            leftRotate.setPosition(intakeRotateTransferWait + intakeRotateLeftOffset);
            rightRotate.setPosition(intakeRotateTransferWait);
            this.rotateState = RotateState.TRANSFERWAIT;
        }
    }

    public void setPivotState(PivotState state) {
        if (state == PivotState.HOVER) {
//            leftPivot.setPosition(intakePivotHover);
            rightPivot.setPosition(intakePivotHover);
            this.pivotState = PivotState.HOVER;
        } else if (state == PivotState.HOVERO) {
//            leftPivot.setPosition(intakePivotHoverOutward);
            double l = intakePivotHoverOutward;
            double r = intakePivotHoverOutwardFull;
            rightPivot.setPosition(extendP*(r-l)+l);
            this.pivotState = PivotState.HOVERO;
        } else if (state == PivotState.GROUND) {
//            leftPivot.setPosition(intakePivotGround);
            rightPivot.setPosition(intakePivotGround);
            this.pivotState = PivotState.GROUND;
        } else if (state == PivotState.GROUNDO) {
//            leftPivot.setPosition(intakePivotGroundOutward);
            double l = intakePivotGroundOutward;
            double r = intakePivotGroundOutwardFull;
            rightPivot.setPosition(extendP*(r-l)+l);
            this.pivotState = PivotState.GROUNDO;
        } else if (state == PivotState.GROUNDOZ) {
//            leftPivot.setPosition(intakePivotGroundOutward+0.02);
            double l = intakePivotGroundOutward;
            double r = intakePivotGroundOutwardFull;
            rightPivot.setPosition(extendP*(r-l)+l+0.03);
            this.pivotState = PivotState.GROUNDOZ;
        } else if (state == PivotState.TRANSFER) {
//            leftPivot.setPosition(intakePivotTransfer);
            rightPivot.setPosition(intakePivotTransfer);
            this.pivotState = PivotState.TRANSFER;
        } else if (state == PivotState.THROWBACK) {
//            leftPivot.setPosition(intakePivotThrowback);
            rightPivot.setPosition(intakePivotThrowback);
            this.pivotState = PivotState.THROWBACK;
        } else if (state == PivotState.HOLDBACK) {
//            leftPivot.setPosition(intakePivotHoldback);
            rightPivot.setPosition(intakePivotHoldback);
            this.pivotState = PivotState.HOLDBACK;
        }
    }

    public void switchGrabState() {
        if (grabState == GrabState.CLOSED || grabState == GrabState.LOOSE) {
            setGrabState(GrabState.OPENFULL);
        } else {
            setGrabState(GrabState.CLOSED);
        }
    }

    public void rotateDegrees(double deg) {
        deg = ((deg % 360) + 360) % 360;
        if (deg > 180) {
            deg -= 360;
        }

        if (deg > 90) {
            deg -= 180;
        } else if (deg < -90) {
            deg += 180;
        }

        this.rotateDegrees = deg;
        setRotateState(rotateState);
    }

    public double getDegrees() {
        return rotateDegrees;
    }

    public void open() {
        setGrabState(GrabState.OPEN);
    }

    public void loose() {
        setGrabState(GrabState.LOOSE);
    }

    public void openfull() {
        setGrabState(GrabState.OPENFULL);
    }

    public void close() {
        setGrabState(GrabState.CLOSED);
    }

    public void transfer() {
        rotateDegrees = 0;
        setRotateState(RotateState.TRANSFERWAIT);
        setPivotState(PivotState.TRANSFER);
        setGrabState(GrabState.CLOSED);
        grabTimer.resetTimer();
        rotateTimer.resetTimer();
    }

//    public void hover() {
//        rotateDegrees = 0;
//        setPivotState(PivotState.HOVER);
//        setRotateState(RotateState.HOVER);
//        setGrabState(GrabState.OPEN);
//        grabTimer.resetTimer();
//    }
    public void hovero(boolean resetdeg, double extendPos) {
        this.extendP = (extendPos-extendZero)/(extendFull-extendZero);
        if (resetdeg)rotateDegrees = 0;
        setPivotState(PivotState.HOVERO);
        setRotateState(RotateState.HOVERO);
        grabTimer.resetTimer();
        setGrabState(GrabState.OPEN);
    }

//    public void ground(boolean resetdeg) {
//        if (pivotState != PivotState.HOVER) {
//            setGrabState(GrabState.OPENFULL);
//        }
//        if (resetdeg)rotateDegrees = 0;
//        setPivotState(PivotState.GROUND);
//        setRotateState(RotateState.GROUND);
//    }

    public void groundo(boolean resetdeg, double extendPos) {
        this.extendP = (extendPos-extendZero)/(extendFull-extendZero);
        if (pivotState != PivotState.HOVERO) {
            setGrabState(GrabState.OPEN);
            grabTimer.resetTimer();
        }
        if (resetdeg)rotateDegrees = 0;
        setPivotState(PivotState.GROUNDO);
        setRotateState(RotateState.GROUNDO);
    }

    public void groundoZ(boolean resetdeg, double extendPos) {
        this.extendP = (extendPos-extendZero)/(extendFull-extendZero);
        if (pivotState != PivotState.HOVERO) {
            setGrabState(GrabState.OPEN);
            grabTimer.resetTimer();
        }
        if (resetdeg)rotateDegrees = 0;
        setPivotState(PivotState.GROUNDOZ);
        setRotateState(RotateState.GROUNDOZ);
    }

    public void throwback() {
        rotateDegrees = 0;
        setPivotState(PivotState.THROWBACK);
        setRotateState(RotateState.THROWBACK);
        setGrabState(GrabState.LOOSE);
    }

    public void throwbackD() {
        rotateDegrees = 0;
        setPivotState(PivotState.THROWBACK);
        setRotateState(RotateState.DODGE);
        setGrabState(GrabState.CLOSED);
    }

    public void holdback() {
        rotateDegrees = 0;
        setPivotState(PivotState.HOLDBACK);
        setRotateState(RotateState.HOLDBACK);
        setGrabState(GrabState.LOOSE);
    }

    public void rotateForward(double b) {
        leftRotate.setPosition(leftRotate.getPosition()+b);
        rightRotate.setPosition(rightRotate.getPosition()+b);
    }

    public void setRotate(double b) {
        leftRotate.setPosition(b + intakeRotateLeftOffset);
        rightRotate.setPosition(b);
    }

    public void setPivot(double b) {
//        leftPivot.setPosition(b);
        rightPivot.setPosition(b);
    }

    public void setGrab(double b) {
        grab.setPosition(b);
    }

    public void init() {
        rotateDegrees = 0;
        setPivotState(PivotState.THROWBACK);
        setRotateState(RotateState.THROWBACK);
        setGrabState(GrabState.OPEN);
    }

    public void start() {
        rotateDegrees = 0;
        setPivotState(PivotState.THROWBACK);
        setRotateState(RotateState.DODGE);
        setGrabState(GrabState.OPEN);
    }

    public void telemetry() {
        telemetry.addData("Intake Rotate State: ", rotateState);
        telemetry.addData("Intake Rotate Pos: ", rightRotate.getPosition());
        telemetry.addData("Intake Rotate Degrees: ", rotateDegrees);
        telemetry.addData("Intake Pivot State: ", pivotState);
        telemetry.addData("Intake Pivot Pos: ", rightPivot.getPosition());
        telemetry.addData("Intake Grab State: ", grabState);
        telemetry.addData("Intake Grab Pos: ", grab.getPosition());
    }

    public void periodic() {
        telemetry();
        if ((this.pivotState == PivotState.HOVER || this.pivotState == PivotState.HOVERO
            || this.pivotState == PivotState.GROUNDO || this.pivotState == PivotState.GROUNDOZ) && this.grabState == GrabState.OPEN && grabTimer.getElapsedTimeSeconds() > 0.2) {
            setGrabState(Intake.GrabState.OPENFULL);
        }
        if ((this.pivotState == PivotState.TRANSFER) && this.grabState == GrabState.CLOSED && grabTimer.getElapsedTimeSeconds() > 0.35) {
            setGrabState(Intake.GrabState.LOOSE);
        }
        if ((this.rotateState == RotateState.TRANSFERWAIT) && rotateTimer.getElapsedTimeSeconds() > 0.5) {
            setRotateState(RotateState.TRANSFER);
        }
    }
}
