package org.firstinspires.ftc.teamcode.opmode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.config.core.subsystems.Lift;

@Config
@TeleOp(group = "Tests", name = "Lift Test")
public class LiftTest extends OpMode {
    public static int target = 0;
    private MultipleTelemetry mtelemetry;

    Lift l;
    Gamepad g1, p1;

    @Override
    public void init() {

        l = new Lift(hardwareMap, telemetry);

        mtelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        g1 = new Gamepad();
        p1 = new Gamepad();
    }

    @Override
    public void loop() {
        p1.copy(g1);
        g1.copy(gamepad1);

        if (g1.a) {
            l.setTarget(target);
        } else {
            l.manual(gamepad1.left_trigger, gamepad1.right_trigger);
        }
//        if (g1.a && !p1.a) {
//            l.toZero();
//        } else if (g1.y && !p1.y) {
//            l.toHighBucket();
//        }
//
//        if (g1.b && !p1.b)
//            o.specimenScore0After();
//
//        if (g1.x && !p1.x)
//            o.specimenScore0();

        l.periodic();
        mtelemetry.addLine("hello2");
        mtelemetry.update();

    }
}
