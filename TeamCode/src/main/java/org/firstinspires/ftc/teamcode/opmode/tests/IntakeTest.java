package org.firstinspires.ftc.teamcode.opmode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.config.core.subsystems.Intake;

@Config
@TeleOp(group = "Tests", name = "Intake Test")
public class IntakeTest extends OpMode {
    public static double target = 0.5, target2 = 0.5;
    private MultipleTelemetry mtelemetry;

    Intake i;
    //    Extend e;
    Gamepad g1, p1;

    @Override
    public void init() {

//        e = new Extend(hardwareMap, telemetry);
//        e.toZero();

        i = new Intake(hardwareMap, telemetry);

        mtelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        g1 = new Gamepad();
        p1 = new Gamepad();
    }

    @Override
    public void loop() {
        p1.copy(g1);
        g1.copy(gamepad1);

        if (g1.a) {
//            i.setRotate(target);
//            i.leftRotate.setPosition(target);
//            i.setGrab(target);
            i.setRotate(target);
        }
        if (g1.dpad_down) {
//            i.leftRotate.setPosition(target2);
//            i.rightRotate.setPosition(target2);
            i.setPivot(target2);
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

        if (g1.y && !p1.y) {
            i.switchGrabState();
        }

//        if (g1.x && !p1.x) {
//            e.switchExtendState();
//        }

        i.periodic();
        mtelemetry.addLine("hello4");
        mtelemetry.addData("target", target);
        mtelemetry.addData("target2", target2);
        mtelemetry.update();

    }
}
