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
@TeleOp(group = "Tests", name = "Full Outtake Test")
public class FullOuttakeTest extends OpMode {
    public static double target = 0, target2 = 0;
    private MultipleTelemetry mtelemetry;

    Outtake o;
    Lift l;
    Gamepad g1, p1;

    @Override
    public void init() {

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
            o.switchGrabState();
        }
        if (g1.left_bumper) {
            o.transfer();
            l.toTransfer();
        }
        if (g1.right_bumper) {
            o.score();
            l.toHighBasket();
        }
        if (g1.b) {
            o.pickup();
            l.toPickup();
        }
        if (g1.x) {
            o.specimen();
            l.toSpecimen();
        }
        //        else {
//            l.manual(gamepad1.left_trigger, gamepad1.right_trigger);
//        }

        l.periodic();
        o.periodic();
        mtelemetry.addLine("hello5");
//        mtelemetry.addData("target",target);
//        mtelemetry.addData("target2",target2);
        mtelemetry.update();

    }
}
