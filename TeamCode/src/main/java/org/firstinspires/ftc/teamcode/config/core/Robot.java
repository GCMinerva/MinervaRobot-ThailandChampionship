package org.firstinspires.ftc.teamcode.config.core;

import static org.firstinspires.ftc.teamcode.config.core.RobotConstants.extendFull;
import static org.firstinspires.ftc.teamcode.config.core.RobotConstants.extendZero;
import static org.firstinspires.ftc.teamcode.config.core.RobotConstants.visionCameraHeight;
import static org.firstinspires.ftc.teamcode.config.core.RobotConstants.visionCameraWidth;
import static org.firstinspires.ftc.teamcode.config.core.RobotConstants.visionExposure;
import static org.firstinspires.ftc.teamcode.config.core.RobotConstants.visionGain;
import static org.firstinspires.ftc.teamcode.config.core.RobotConstants.visionWhiteBalance;
import static org.firstinspires.ftc.teamcode.config.core.paths.TeleOpPaths.pickup;
import static org.firstinspires.ftc.teamcode.config.core.paths.TeleOpPaths.spec;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.controller.PIDController;
import com.pedropathing.commands.FollowPath;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.FocusControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl;
import org.firstinspires.ftc.teamcode.config.commands.DoPath;
import org.firstinspires.ftc.teamcode.config.commands.ForwardChamber;
import org.firstinspires.ftc.teamcode.config.commands.GenPath;
import org.firstinspires.ftc.teamcode.config.commands.Hang;
import org.firstinspires.ftc.teamcode.config.commands.ProcessFrame;
import org.firstinspires.ftc.teamcode.config.commands.Transport;
import org.firstinspires.ftc.teamcode.config.commands.TransportLow;
import org.firstinspires.ftc.teamcode.config.core.paths.FourSamp;
import org.firstinspires.ftc.teamcode.config.core.paths.TeleOpPaths;
import org.firstinspires.ftc.teamcode.config.core.subsystems.Extend;
import org.firstinspires.ftc.teamcode.config.core.subsystems.Intake;
import org.firstinspires.ftc.teamcode.config.core.subsystems.Lift;
import org.firstinspires.ftc.teamcode.config.core.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.config.core.util.Alliance;
import org.firstinspires.ftc.teamcode.config.core.util.Color;
import org.firstinspires.ftc.teamcode.config.core.vision.Vision;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

public class Robot {

    public enum RobotState {
        IDLE, GROUND, HOVER, TRANSFER, SCORE, PICKUP, SPECIMEN, THROWBACK
    }

    RobotState robotState = RobotState.IDLE;
    private HardwareMap h;
    private MultipleTelemetry t;
    private Gamepad g1a, g2a, g1, g2, p1, p2;
    private Alliance a;
    private Follower f;
    private Extend e;
    private Intake i;
    private Lift l;
    private Outtake o;
    private Vision v;
    private ServoImplEx pushUp;

    private OpenCvWebcam camera;
    private boolean rc = true;
    public static Pose autoEndPose = new Pose();

    public Pose s = new Pose();
    public static double currentHeading = 0;
    private PIDController headingPIDController = new PIDController(2, 0, 0.2);
    public Timer hPidTimer, rBumperTimer, lBumperTimer;
    private boolean rBumperQueued, lBumperQueued;
    public double speed = 0.9;
    public double slow = 0.35;
    private boolean drive;

    public Robot(HardwareMap h, Telemetry t, Gamepad g1a, Gamepad g2a, Alliance a, Pose startPose, boolean robotCentric) {
        this.h = h;
        this.t = new MultipleTelemetry(t, FtcDashboard.getInstance().getTelemetry());
        this.g1a = g1a;
        this.g2a = g2a;
        this.a = a;
        this.rc = robotCentric;

        f = new Follower(this.h, FConstants.class, LConstants.class);
        f.setStartingPose(startPose);

        setRobotState(RobotState.IDLE);

        e = new Extend(this.h, this.t);
        i = new Intake(this.h, this.t);
        l = new Lift(this.h, this.t);
        o = new Outtake(this.h, this.t);
        pushUp = this.h.get(ServoImplEx.class, "f");

        this.g1 = new Gamepad();
        this.g2 = new Gamepad();
        this.p1 = new Gamepad();
        this.p2 = new Gamepad();

        hPidTimer = new Timer();
        rBumperTimer = new Timer();
        lBumperTimer = new Timer();
        rBumperQueued = false;
        lBumperQueued = false;
    }

    public Robot(HardwareMap h, Telemetry t, Alliance a, Pose startPose) {
        this.h = h;
        this.t = new MultipleTelemetry(t, FtcDashboard.getInstance().getTelemetry());
        this.a = a;
        this.s = startPose.copy();

        f = new Follower(this.h, FConstants.class, LConstants.class);
        f.setStartingPose(startPose);

        e = new Extend(this.h, this.t);
        l = new Lift(this.h, this.t);
        i = new Intake(this.h, this.t);
        o = new Outtake(this.h, this.t);

        t.addData("s", s);
        t.addData("status", "not ready");
    }

    public void initVision(Color color) {
        int cameraMonitorViewId = this.getH().appContext.getResources().getIdentifier("cameraMonitorViewId", "id",
            this.getH().appContext.getPackageName());

        camera = OpenCvCameraFactory.getInstance().createWebcam(this.getH().get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        v = new Vision(t,color);

        camera.openCameraDeviceAsync(new OpenCvWebcam.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.getExposureControl().setMode(ExposureControl.Mode.Manual);
                camera.getExposureControl().setExposure(visionExposure, TimeUnit.MILLISECONDS);
                camera.getGainControl().setGain(visionGain);
                camera.getWhiteBalanceControl().setMode(WhiteBalanceControl.Mode.MANUAL);
                camera.getWhiteBalanceControl().setWhiteBalanceTemperature(visionWhiteBalance);
                camera.getFocusControl().setMode(FocusControl.Mode.Fixed);
                camera.setPipeline(v);
                camera.startStreaming(visionCameraWidth, visionCameraHeight,
                    OpenCvCameraRotation.UPRIGHT, OpenCvWebcam.StreamFormat.MJPEG);
                FtcDashboard.getInstance().startCameraStream(camera, 0);
            }

            @Override
            public void onError(int errorCode) {
                t.addData("Camera Error", errorCode);
                t.update();
            }
        });
    }

    public void aPeriodic() {
        t.addData("path", f.getCurrentPath());
        t.addData("t value", f.getCurrentTValue());
        t.addData("path num", f.getCurrentPathNumber());
        t.addData("Pose X: ",f.getPose().getX());
        t.addData("Pose Y: ",f.getPose().getY());
//        f.telemetryDebug(t);

        e.periodic();
        l.periodic();
        i.periodic();
        o.periodic();
        if (v != null)v.periodic();
        f.update();
        t.update();
    }
    public void aInitLoop(Gamepad g2a) {
        t.update();
    }

    public void tStart() {
        i.start();
        e.start();
        o.start();
        l.start();
        TeleOpPaths.score = FourSamp.score3;
        drive = true;
        f.startTeleopDrive();
        pushUp.setPosition(0.3);
        currentHeading = f.getPose().getHeading();
        setRobotState(RobotState.IDLE);
    }

    public void tTelemetry() {
        t.addData("Pose X: ",f.getPose().getX());
        t.addData("Pose Y: ",f.getPose().getY());
        t.addData("Pose Heading: ",f.getPose().getHeading());
        t.addData("Current Heading: ",currentHeading);
        t.addData("Push Pos: ",pushUp.getPosition());
        t.addData("Robot State: ", robotState);
    }

    public void tPeriodic() { // teleop
        e.periodic();
        l.periodic();
        i.periodic();
        o.periodic();
        tTelemetry();
        f.update();
        t.update();
    }

    public void setRobotState(RobotState robotState) {
        this.robotState = robotState;
    }

    public List<Command> soloControls() {
        List<Command> commands = new ArrayList<>();
        speed = 1;

        p1.copy(g1);
        p2.copy(g2);
        g1.copy(g1a);
        g2.copy(g2a);

//        if (g1.a && !p1.a) {
//            if (this.robotState == RobotState.HOVER) {
//                i.groundo();
//                robotState = RobotState.GROUND;
//            } else {
//                i.switchGrabState();
//            }
//        }
//        if (g1.y && !p1.y) {
//            o.switchGrabState();
//        }
//        if (g1.dpad_left && !p1.dpad_left) {
//            if (this.robotState == RobotState.HOVER || this.robotState == RobotState.GROUND) {
//                i.rotateDegrees(i.rotateDegrees + 15);
//            } else {
//                commands.add(new GrabSpec1(this));
//            }
//        }
//        if (g1.dpad_right && !p1.dpad_right) {
//            if (this.robotState == RobotState.HOVER || this.robotState == RobotState.GROUND) {
//                i.rotateDegrees(i.rotateDegrees - 15);
//            }
//        }
//        if (g1.dpad_up && !p1.dpad_up) {
//            if (this.robotState == RobotState.IDLE) {
//                commands.add(new Hover(this));
//            } else if (this.robotState == RobotState.HOVER) {
//                commands.add(new Idle(this));
//            }
//        }
//        if (g1.left_bumper && !p1.left_bumper) {
//            if (this.robotState == RobotState.HOVER || this.robotState == RobotState.GROUND) {
//                commands.add(new Transport(this));
//            } else {
//                commands.add(new Idle(this));
//                o.specimen();
//            }
//        }
//        if (g1.right_bumper && !p1.right_bumper) {
//            if (this.robotState == RobotState.HOVER || this.robotState == RobotState.GROUND) {
//                setRobotState(RobotState.PICKUP);
//                i.throwback();
//                o.pickup();
//                l.toPickup();
//                e.toZero();
//            } else {
//                commands.add(new Idle(this));
//                e.toGrab();
//                l.toSpecimen();
//                o.specimen();
//            }
//        }
        if (g1.left_bumper && !g1.right_bumper) {
            lBumperTimer.resetTimer();
            lBumperQueued = true;
        }
        if (lBumperTimer.getElapsedTimeSeconds() < 0.1 && lBumperQueued && !g1.right_bumper) {
            if (g1.x && !p1.x) {
                if (robotState == RobotState.SCORE) {
                    o.pickup2();
                } else {
                    o.pickup();
                }
                setRobotState(RobotState.PICKUP);
                e.toZero();
                i.throwbackD();
                l.toPickup();
                commands.add(new SequentialCommandGroup(
                    new WaitCommand(500),
                    new InstantCommand(() -> this.getI().setRotateState(Intake.RotateState.THROWBACK))
                ));
//                o.pickup();
                lBumperQueued = false;
            }
            if (g1.y && !p1.y) {
                setRobotState(RobotState.SCORE);
                commands.add(new Transport(this));
                lBumperQueued = false;
            }
            if (g1.b && !p1.b) {
                setRobotState(RobotState.SCORE);
                commands.add(new TransportLow(this));
                lBumperQueued = false;
            }
            if (g1.a && !p1.a) {
                setRobotState(RobotState.SPECIMEN);
                i.throwbackD();
                i.open();
                o.close();
//                o.specimen();
                commands.add(new SequentialCommandGroup(
                    new WaitCommand(200),
                    new InstantCommand(() -> this.o.specimen())
                ));
                l.toSpecimen();
                lBumperQueued = false;
            }
//            if (g1.dpad_right && !p1.dpad_right) {
//                setRobotState(RobotState.SCORE);
//                commands.add(new Transport(this)
//                    .alongWith(new SequentialCommandGroup(
//                        new WaitCommand(500),
//                        new ScorePath(this,score)
//                    ))
//                );
//                lBumperQueued = false;
//            }
        }
        if (g1.right_bumper && !g1.left_bumper) {
            if (!p1.right_bumper) {
                if (this.o.pivotState == Outtake.PivotState.SPECIMEN) {
                    o.openfull();
                }
                if (robotState == RobotState.GROUND) {
                    commands.add(new InstantCommand(() -> {
                        i.setRotateState(Intake.RotateState.HOVERO);
                        i.setPivotState(Intake.PivotState.HOVERO);
                    }));
                    setRobotState(RobotState.HOVER);
                } else if (robotState != RobotState.HOVER) {
                    commands.add(new InstantCommand(() -> i.hovero(true,extendFull)));
                    i.open();
                    setRobotState(RobotState.HOVER);
                }
            }
        } else if (g1.left_bumper && !g1.right_bumper) {
        } else if (g1.left_bumper && g1.right_bumper) {
            if (g1.start && !p1.start) {
                f.setPose(new Pose(f.getPose().getX(), f.getPose().getY(), 0));
                currentHeading = 0;
            }
//            if (g1.x && !p1.x) {
//                drive = false;
//                e.toZero();
//                i.throwback();
//                this.f.setPose(pickup);
//                currentHeading = 0;
//                commands.add(
//                    new SequentialCommandGroup(
//                        new ParallelCommandGroup(
//                            new WaitCommand(150)
//                                .andThen(new FollowPath(this.getF(), spec()).setCompletionThreshold(0.95)),
//                            new ForwardChamber(this)
//                        ),
//                        new ParallelCommandGroup(
//                            new FollowPath(this.getF(), pickup()).setCompletionThreshold(0.99),
//                            new InstantCommand(() -> {
//                                this.getO().pickup();
//                            })
//                        ),
//                        new WaitCommand(200),
//                        new InstantCommand(() -> {
//                            this.getF().startTeleopDrive();
//                            this.setRobotState(RobotState.PICKUP);
//                        })
//                    )
//                );
//            }
//            if (g1.b && !p1.b) {
//                drive = false;
//                e.toZero();
//                i.throwback();
//                this.f.setPose(pickup);
//                currentHeading = 0;
//                commands.add(
//                    new SequentialCommandGroup(
//                        new ParallelCommandGroup(
//                            new WaitCommand(150)
//                                .andThen(new FollowPath(this.getF(), spec()).setCompletionThreshold(0.95)),
//                            new ForwardChamber(this)
//                        ),
//                        new WaitCommand(100),
//                        new InstantCommand(() -> {
//                            this.getF().startTeleopDrive();
//                            this.setRobotState(RobotState.IDLE);
//                        })
//                    )
//                );
//            }
        } else {
            if (g1.a && !p1.a && (rBumperTimer.getElapsedTimeSeconds() >= 0.1 || !rBumperQueued)) {
                if (this.robotState != RobotState.HOVER && (i.pivotState == Intake.PivotState.HOVER || i.pivotState == Intake.PivotState.HOVERO)) {
                    this.robotState = RobotState.HOVER;
                }
                if (this.robotState != RobotState.GROUND && (i.pivotState == Intake.PivotState.GROUND || i.pivotState == Intake.PivotState.GROUNDO)) {
                    this.robotState = RobotState.GROUND;
                }
                if (i.grabState == Intake.GrabState.CLOSED && this.robotState == RobotState.HOVER) {
                    i.switchGrabState();
                } else if (this.robotState == RobotState.HOVER) {
                    i.groundo(false,e.getPos());
                    robotState = RobotState.GROUND;
//                } else if (i.rotateState == Intake.RotateState.DODGE) {
//                    commands.add(
//                        new SequentialCommandGroup(
//                            new InstantCommand(() -> this.getI().setRotateState(Intake.RotateState.THROWBACK)),
//                            new WaitCommand(100),
//                            new InstantCommand(() -> this.getI().open())
//                        )
//                    );
//                    i.setRotateState(Intake.RotateState.THROWBACK);
                } else {
                    i.switchGrabState();
                }
            }
            if (g1.x && !p1.x) {
                i.rotateDegrees(i.getDegrees() + 30);
            }
            if (g1.b && !p1.b) {
                i.rotateDegrees(i.getDegrees() - 30);
            }
            if (g1.y && !p1.y && (rBumperTimer.getElapsedTimeSeconds() >= 0.1 || !rBumperQueued)) {
                o.switchGrabState();
                if (o.grabState == Outtake.GrabState.OPENFULL && robotState == RobotState.SCORE) {
                    TeleOpPaths.score = this.getF().getPose();
                }
                if (this.robotState == RobotState.PICKUP && i.grabState == Intake.GrabState.CLOSED) {
                    i.open();
                }
            }
            if (g1.dpad_up && !p1.dpad_up) {
                e.setTarget(e.getPos()-0.02);
            }
            if (g1.dpad_down && !p1.dpad_down) {
                e.setTarget(e.getPos()+0.02);
            }
            if (g1.start && !p1.start) {
//                if (CVPaths.paths.isEmpty()) {
                if (this.getV().getBestSample(-10, 3, 0, false) != null) {
                    commands.add(new SequentialCommandGroup(
                        new GenPath(this, 0, -10, 3, true),
                        new DoPath(this, true, false, true),
                        new InstantCommand(() -> {
                            currentHeading = f.getPose().getHeading();
                            f.startTeleopDrive();
                        })
                    ));
                } else {
                    commands.add(new SequentialCommandGroup(
                        new WaitUntilCommand(() -> this.getF().getVelocity().getMagnitude() < 0.5),
                        new ProcessFrame(this, Color.YELLOW, 2),
                        new WaitUntilCommand(() -> this.getV().cnt2 > 0),
                        new GenPath(this, 0, -10, 3, true),
                        new DoPath(this, true, false, true),
                        new InstantCommand(() -> {
                            currentHeading = f.getPose().getHeading();
                            f.startTeleopDrive();
                        })
                    ));
                }
            }
            if (g1.back && !p1.back) {
                commands.add(new SequentialCommandGroup(
                    new WaitUntilCommand(() -> this.getF().getVelocity().getMagnitude() < 0.5),
                    new ProcessFrame(this, Color.YELLOW, 2),
                    new WaitUntilCommand(() -> this.getV().cnt2 > 0),
                    new GenPath(this, 0, -10, 3, true),
                    new DoPath(this, true, false, true),
                    new InstantCommand(() -> {
                        currentHeading = f.getPose().getHeading();
                        f.startTeleopDrive();
                    })
                ));
            }
            if (g1.right_stick_button) {
                currentHeading = 0;
            }
        }
        if (g1.right_bumper && !g1.left_bumper) {
            rBumperTimer.resetTimer();
            rBumperQueued = true;
        }
        if (rBumperTimer.getElapsedTimeSeconds() < 0.1 && rBumperQueued && !g1.left_bumper) {
            if (g1.a && !p1.a) {
                e.toZero();
                i.extendP = (e.getPos()-extendZero)/(extendFull-extendZero);
                commands.add(new InstantCommand(() -> {
                    i.setRotateState(Intake.RotateState.HOVERO);
                    i.setPivotState(Intake.PivotState.HOVERO);
                }));
                setRobotState(RobotState.HOVER);
                rBumperQueued = false;
            }
            if (g1.b && !p1.b) {
                e.toFirst();
                i.extendP = (e.getPos()-extendZero)/(extendFull-extendZero);
                commands.add(new InstantCommand(() -> {
                    i.setRotateState(Intake.RotateState.HOVERO);
                    i.setPivotState(Intake.PivotState.HOVERO);
                }));
                setRobotState(RobotState.HOVER);
                rBumperQueued = false;
            }
            if (g1.x && !p1.x) {
                e.toSecond();
                i.extendP = (e.getPos()-extendZero)/(extendFull-extendZero);
                commands.add(new InstantCommand(() -> {
                    i.setRotateState(Intake.RotateState.HOVERO);
                    i.setPivotState(Intake.PivotState.HOVERO);
                }));
                setRobotState(RobotState.HOVER);
                rBumperQueued = false;
            }
            if (g1.y && !p1.y) {
                e.toFull();
                i.extendP = (e.getPos()-extendZero)/(extendFull-extendZero);
                commands.add(new InstantCommand(() -> {
                    i.setRotateState(Intake.RotateState.HOVERO);
                    i.setPivotState(Intake.PivotState.HOVERO);
                }));
                setRobotState(RobotState.HOVER);
                rBumperQueued = false;
            }
        }

//        if (g1.start) {
//            f.setPose(new Pose(f.getPose().getX(), f.getPose().getY(), 0));
//            currentHeading = f.getPose().getHeading();
//        }

//        double addx = ((g1.dpad_left ? 1 : 0)-(g1.dpad_right ? 1 : 0))*slow*1;
//        double addy = ((g1.dpad_up ? 1 : 0)-(g1.dpad_down ? 1 : 0))*slow*1;
        double addx = 0;
        double addy = 0;

        if (g1.ps && !p1.ps) {
            drive = true;
            f.startTeleopDrive();
        }

        double mult = 1-0.7*g1.left_trigger;

        double xpower = 0, ypower = 0, zpower = 0;

        if (g1.right_stick_x != 0 || hPidTimer.getElapsedTimeSeconds() < 0.2) {
            xpower = -g1.left_stick_y * speed * mult + addy;
            ypower = -g1.left_stick_x * speed * mult + addx;
            zpower = -g1.right_stick_x * speed * mult * 0.5;
//            f.setTeleOpMovementVectors(-g1.left_stick_y * speed * mult + addy, -g1.left_stick_x * speed * mult + addx, -g1.right_stick_x * speed * mult * 0.5, rc);
            currentHeading = f.getPose().getHeading();
            if (g1.right_stick_x != 0) hPidTimer.resetTimer();
        } else if (pushUp.getPosition() != 1) {
            double headingError = f.getPose().getHeading() - currentHeading;
            headingError = Math.IEEEremainder(headingError, 2 * Math.PI);

            double headingCorrection;
            if (Math.abs(headingError) < Math.toRadians(2)) {
                headingCorrection = 0;
            } else {
                headingCorrection = headingPIDController.calculate(headingError);
            }
            t.addData("heading", currentHeading);
            t.addData("headingError", headingError);
            t.addData("headingCorrection", headingCorrection);
            xpower = -g1.left_stick_y * speed * mult + addy;
            ypower = -g1.left_stick_x * speed * mult + addx;
            zpower = headingCorrection;
//            f.setTeleOpMovementVectors(-g1.left_stick_y * speed * mult + addy, -g1.left_stick_x * speed * mult + addx, headingCorrection, rc);
        }
        double sin = Math.sin(currentHeading);
        double cos = Math.cos(currentHeading);
        if (g1.left_trigger > 0.5) {
            f.setTeleOpMovementVectors(xpower * (cos * cos + 1.7 * sin * sin) - 0.7 * ypower * cos * sin,
                ypower * (1.7 * cos * cos + sin * sin) - 0.7 * xpower * cos * sin, zpower, rc);
        } else {
            f.setTeleOpMovementVectors(xpower, ypower, zpower, rc);
        }
        if (g1.left_bumper && g1.ps && !p1.ps) {
            commands.add(new Hang(this));
        }
        if (g1.left_bumper && g1.back && !p1.back) {
            l.toHang();
            o.specimen();
        }
        return commands;
    }
    public List<Command> dualControls() {
        List<Command> commands = new ArrayList<>();
        speed = 0.9;

        p1.copy(g1);
        p2.copy(g2);
        g1.copy(g1a);
        g2.copy(g2a);

//        if (g1.a && !p1.a) {
//            if (this.robotState == RobotState.HOVER) {
//                i.groundo();
//                robotState = RobotState.GROUND;
//            } else {
//                i.switchGrabState();
//            }
//        }
//        if (g1.y && !p1.y) {
//            o.switchGrabState();
//        }
//        if (g1.dpad_left && !p1.dpad_left) {
//            if (this.robotState == RobotState.HOVER || this.robotState == RobotState.GROUND) {
//                i.rotateDegrees(i.rotateDegrees + 15);
//            } else {
//                commands.add(new GrabSpec1(this));
//            }
//        }
//        if (g1.dpad_right && !p1.dpad_right) {
//            if (this.robotState == RobotState.HOVER || this.robotState == RobotState.GROUND) {
//                i.rotateDegrees(i.rotateDegrees - 15);
//            }
//        }
//        if (g1.dpad_up && !p1.dpad_up) {
//            if (this.robotState == RobotState.IDLE) {
//                commands.add(new Hover(this));
//            } else if (this.robotState == RobotState.HOVER) {
//                commands.add(new Idle(this));
//            }
//        }
//        if (g1.left_bumper && !p1.left_bumper) {
//            if (this.robotState == RobotState.HOVER || this.robotState == RobotState.GROUND) {
//                commands.add(new Transport(this));
//            } else {
//                commands.add(new Idle(this));
//                o.specimen();
//            }
//        }
//        if (g1.right_bumper && !p1.right_bumper) {
//            if (this.robotState == RobotState.HOVER || this.robotState == RobotState.GROUND) {
//                setRobotState(RobotState.PICKUP);
//                i.throwback();
//                o.pickup();
//                l.toPickup();
//                e.toZero();
//            } else {
//                commands.add(new Idle(this));
//                e.toGrab();
//                l.toSpecimen();
//                o.specimen();
//            }
//        }
        if (g2.right_bumper)rBumperTimer.resetTimer();
        if (rBumperTimer.getElapsedTimeSeconds() < 0.3) {
            if (g2.a && !p2.a) {
                e.toZero();
            }
            if (g2.b && !p2.b) {
                e.toFirst();
            }
            if (g2.x && !p2.x) {
                e.toSecond();
            }
            if (g2.y && !p2.y) {
                e.toFull();
            }
        }
        if (g2.left_bumper)lBumperTimer.resetTimer();
        if (lBumperTimer.getElapsedTimeSeconds() < 0.3) {
        }
        if (g1.right_bumper && !g1.left_bumper) {
        } else if (g1.left_bumper && !g1.right_bumper) {
        } else if (g1.left_bumper && g1.right_bumper) {
            if (g1.x && !p1.x) {
                drive = false;
                e.toZero();
                i.throwback();
                this.f.setPose(pickup);
                commands.add(
                    new SequentialCommandGroup(
                        new ParallelCommandGroup(
                            new WaitCommand(150)
                                .andThen(new FollowPath(this.getF(), spec()).setCompletionThreshold(0.95)),
                            new ForwardChamber(this)
                        ),
                        new ParallelCommandGroup(
                            new FollowPath(this.getF(), pickup()).setCompletionThreshold(0.99),
                            new InstantCommand(() -> {
                                this.getO().pickup();
                            })
                        ),
                        new WaitCommand(200),
                        new InstantCommand(() -> {
                            this.getF().startTeleopDrive();
                            this.setRobotState(RobotState.PICKUP);
                        })
                    )
                );
            }
            if (g1.b && !p1.b) {
                drive = false;
                e.toZero();
                i.throwback();
                this.f.setPose(pickup);
                commands.add(
                    new SequentialCommandGroup(
                        new ParallelCommandGroup(
                            new WaitCommand(150)
                                .andThen(new FollowPath(this.getF(), spec()).setCompletionThreshold(0.95)),
                            new ForwardChamber(this)
                        ),
                        new WaitCommand(100),
                        new InstantCommand(() -> {
                            this.getF().startTeleopDrive();
                            this.setRobotState(RobotState.IDLE);
                        })
                    )
                );
            }
        } else {
        }
        if (g2.dpad_down && !p2.dpad_down) {
            setRobotState(RobotState.PICKUP);
            e.toZero();
            i.throwbackD();
            l.toPickup();
            o.pickup();
        } else if (g2.dpad_up && !p2.dpad_up) {
            setRobotState(RobotState.SCORE);
            commands.add(new Transport(this));
        }

        if (g2.right_bumper && !g2.left_bumper) {
            if (!p2.right_bumper) {
                if (robotState == RobotState.GROUND) {
                    commands.add(new InstantCommand(() -> {
                        i.setRotateState(Intake.RotateState.HOVERO);
                        i.setPivotState(Intake.PivotState.HOVERO);
                    }));
                    setRobotState(RobotState.HOVER);
                } else if (robotState != RobotState.HOVER) {
                    commands.add(new InstantCommand(() -> i.hovero(true,extendFull)));
                    setRobotState(RobotState.HOVER);
                }
            }
        } else if (g2.left_bumper && !g2.right_bumper) {
        } else if (g2.left_bumper && g2.right_bumper) {
        } else {
            if (g2.a && !p2.a) {
                if (this.robotState != RobotState.HOVER && (i.pivotState == Intake.PivotState.HOVER || i.pivotState == Intake.PivotState.HOVERO)) {
                    this.robotState = RobotState.HOVER;
                }
                if (this.robotState == RobotState.HOVER) {
                    i.groundo(false,extendFull);
                    robotState = RobotState.GROUND;
                } else if (i.rotateState == Intake.RotateState.DODGE) {
                    commands.add(
                        new SequentialCommandGroup(
                            new InstantCommand(() -> this.getI().setRotateState(Intake.RotateState.THROWBACK)),
                            new WaitCommand(30),
                            new InstantCommand(() -> this.getI().open())
                        )
                    );
                    i.setRotateState(Intake.RotateState.THROWBACK);
                } else {
                    i.switchGrabState();
                }
            }
            if (g2.x && !p2.x) {
                i.rotateDegrees(i.getDegrees() + 30);
            }
            if (g2.b && !p2.b) {
                i.rotateDegrees(i.getDegrees() - 30);
            }
            if (g2.y && !p2.y) {
                o.switchGrabState();
            }
            if (g2.start) {
                currentHeading = 0;
            }
        }

        if (g1.right_stick_button) {
            f.setPose(new Pose(f.getPose().getX(), f.getPose().getY(), 0));
            currentHeading = f.getPose().getHeading();
        }

        double addx = ((g1.x ? 1 : 0)-(g1.b ? 1 : 0))*slow*2.5;
        double addy = ((g1.y ? 1 : 0)-(g1.a ? 1 : 0))*slow*1;
        double addtheta = ((g1.dpad_left ? 1 : 0)-(g1.dpad_right ? 1 : 0))*slow*2.5;

        if (g1.ps && !p1.ps) {
            drive = true;
            f.startTeleopDrive();
        }

        if ((g1.left_stick_x != 0 || addtheta != 0) || hPidTimer.getElapsedTimeSeconds() < 0.2) {
            f.setTeleOpMovementVectors(-g1.left_stick_y * speed + addy, -g1.right_stick_x * speed + addx, (-g1.left_stick_x * speed + addtheta) * 0.5, rc);
            currentHeading = f.getPose().getHeading();
            if (g1.left_stick_x != 0 || addtheta != 0) hPidTimer.resetTimer();
        } else if (pushUp.getPosition() != 1) {
            double headingError = f.getPose().getHeading() - currentHeading;
            headingError = Math.IEEEremainder(headingError, 2 * Math.PI);

            double headingCorrection;
            if (Math.abs(headingError) < Math.toRadians(2)) {
                headingCorrection = 0;
            } else {
                headingCorrection = headingPIDController.calculate(headingError);
            }
            t.addData("heading", currentHeading);
            t.addData("headingError", headingError);
            t.addData("headingCorrection", headingCorrection);
            f.setTeleOpMovementVectors(-g1.left_stick_y * speed + addy, -g1.right_stick_x * speed + addx, headingCorrection, rc);
        }

        if (g1.left_bumper && g1.back && !p1.back) {
            commands.add(new Hang(this));
        }
        return commands;
    }

    public HardwareMap getH() {
        return h;
    }
    public Follower getF() {
        return f;
    }

    public Extend getE() {
        return e;
    }

    public Intake getI() {
        return i;
    }

    public Lift getL() {
        return l;
    }

    public Outtake getO() {
        return o;
    }

    public Telemetry getT() {
        return t;
    }

    public Vision getV() {
        return v;
    }
    public ServoImplEx getP() {
        return pushUp;
    }
}
