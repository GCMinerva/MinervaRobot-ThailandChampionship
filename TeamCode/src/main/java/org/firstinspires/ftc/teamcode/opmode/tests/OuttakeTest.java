package org.firstinspires.ftc.teamcode.opmode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.config.core.subsystems.Lift;
import org.firstinspires.ftc.teamcode.config.core.subsystems.Outtake;

@Config
@TeleOp(group = "Tests", name = "Outtake Test")
public class OuttakeTest extends OpMode {
    public static double target = 0.5, target2 = 0.5;
    private MultipleTelemetry mtelemetry;

    Outtake o;
    Lift l;
    //    Intake i;
    Gamepad g1, p1;

    @Override
    public void init() {

//        i = new Intake(hardwareMap, telemetry);
        o = new Outtake(hardwareMap, telemetry);
        l = new Lift(hardwareMap, telemetry);
        l.toTransfer();

        mtelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        g1 = new Gamepad();
        p1 = new Gamepad();
    }

    @Override
    public void loop() {
        p1.copy(g1);
        g1.copy(gamepad1);

        if (g1.y && !p1.y) {
            o.switchGrabState();
        }
        if (g1.a) {
//            o.leftPivot.setPosition(target);
//            o.setPivot(target);
            o.setGrab(target);
//            o.grab.setPosition(target);
//            i.setGrab(target);
        }
//        if (g1.left_bumper)i.transfer();
//        if (g1.right_bumper)i.hover();
        if (g1.dpad_down) {
            o.rotate.setPosition(target2);
//            o.rightPivot.setPosition(target2);
        }
        //        else {
//            l.manual(gamepad1.left_trigger, gamepad1.right_trigger);
//        }
//
        if (g1.b && !p1.b)
            target += 0.01*(g1.left_bumper ? 10 : 1);

        if (g1.x && !p1.x)
            target -= 0.01*(g1.left_bumper ? 10 : 1);

        if (g1.dpad_right && !p1.dpad_right)
            target2 += 0.01*(g1.left_bumper ? 10 : 1);

        if (g1.dpad_left && !p1.dpad_left)
            target2 -= 0.01*(g1.left_bumper ? 10 : 1);


//        if (g1.x && !p1.x) {
//            e.switchExtendState();
//        }

        o.periodic();
        l.periodic();
        mtelemetry.addLine("hello5");
        mtelemetry.addData("target", target);
        mtelemetry.addData("target2", target2);
        mtelemetry.update();
    }
}
