package org.firstinspires.ftc.teamcode.opmode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.config.core.subsystems.Extend;
import org.firstinspires.ftc.teamcode.config.core.subsystems.Intake;
import org.firstinspires.ftc.teamcode.config.core.subsystems.Lift;
import org.firstinspires.ftc.teamcode.config.core.subsystems.Outtake;

@Config
@TeleOp(group = "Tests", name = "Full Test")
public class FullTest extends OpMode {
    public static double target = 0, target2 = 0;
    private MultipleTelemetry mtelemetry;

    Intake i;
    Extend e;
    Lift l;
    Outtake o;
    Gamepad g1, p1;

    @Override
    public void init() {

        e = new Extend(hardwareMap, telemetry);
        e.toZero();

        i = new Intake(hardwareMap, telemetry);
        l = new Lift(hardwareMap, telemetry);
        o = new Outtake(hardwareMap, telemetry);

        mtelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        g1 = new Gamepad();
        p1 = new Gamepad();
    }

    @Override
    public void loop() {
        p1.copy(g1);
        g1.copy(gamepad1);

        if (g1.a && !p1.a) {
            i.switchGrabState();
        }
        if (g1.y && !p1.y) {
            o.switchGrabState();
        }
        if (i.rotateState == Intake.RotateState.HOVER) {
            if (g1.dpad_left && !p1.dpad_left) {
                i.rotateDegrees(i.rotateDegrees + 15);
            }
            if (g1.dpad_right && !p1.dpad_right) {
                i.rotateDegrees(i.rotateDegrees - 15);
            }
        }
        if (g1.left_bumper && !p1.left_bumper) {
            i.transfer();
            o.transfer();
            l.toTransfer();
            e.toZero();
        }
        if (g1.right_bumper && !p1.right_bumper) {
            if (i.rotateState == Intake.RotateState.HOVER) {
                i.throwback();
                o.pickup();
                l.toPickup();
                e.toZero();
            } else {
//                i.hover();
//                e.toFull();
                e.toZero();
                i.throwback();
                l.toSpecimen();
                o.specimen();
            }
        }
        //        else {
//            l.manual(gamepad1.left_trigger, gamepad1.right_trigger);
//        }
//
//        if (g1.b && !p1.b)
//            target += 0.01;
//
//        if (g1.x && !p1.x)
//            target -= 0.01;
//
//        if (g1.dpad_right && !p1.dpad_right)
//            target2 += 0.01;
//
//        if (g1.dpad_left && !p1.dpad_left)
//            target2 -= 0.01;


//        if (g1.x && !p1.x) {
//            e.switchExtendState();
//        }

        i.periodic();
        e.periodic();
        l.periodic();
        o.periodic();
        mtelemetry.addLine("hello5");
//        mtelemetry.addData("target",target);
//        mtelemetry.addData("target2",target2);
        mtelemetry.update();

    }
}
