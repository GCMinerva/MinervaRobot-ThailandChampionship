package org.firstinspires.ftc.teamcode.opmode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.config.core.subsystems.Extend;
import org.firstinspires.ftc.teamcode.config.core.subsystems.Intake;

@Config
@TeleOp(group = "Tests", name = "Extend Test")
public class ExtendTest extends OpMode {
    public static double target = 0.5;
    private MultipleTelemetry mtelemetry;

    Extend e;
    Intake i;
    Gamepad g1, p1;

    @Override
    public void init() {

        e = new Extend(hardwareMap, telemetry);
        e.toZero();
        i = new Intake(hardwareMap, telemetry);
//        i.hovero(true,e.getPos());
//        i.openfull();
        i.groundo(true,e.getPos());
        i.close();

        mtelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        g1 = new Gamepad();
        p1 = new Gamepad();
    }

    @Override
    public void loop() {
        p1.copy(g1);
        g1.copy(gamepad1);

        if (g1.a)
            e.setTarget(target);
//            e.toInches(target*100);
//        else {
//            l.manual(gamepad1.left_trigger, gamepad1.right_trigger);
//        }
//
        if (g1.b && !p1.b)
            target += 0.001;

        if (g1.x && !p1.x)
            target -= 0.001;
//
        if (g1.dpad_right && !p1.dpad_right)
            target += 0.01;

        if (g1.dpad_left && !p1.dpad_left)
            target -= 0.01;

        if (g1.y && !p1.y) {
//            e.switchExtendState();
            e.toZero();
        }

        e.periodic();
        i.periodic();
        mtelemetry.addLine("hello");
        mtelemetry.addData("target", target);
        mtelemetry.update();

    }
}
