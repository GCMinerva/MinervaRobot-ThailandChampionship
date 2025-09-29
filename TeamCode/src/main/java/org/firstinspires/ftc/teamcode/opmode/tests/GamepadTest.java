package org.firstinspires.ftc.teamcode.opmode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

@Config
@TeleOp(group = "Tests", name = "Gamepad Test")
public class GamepadTest extends OpMode {
    //Talk Volatile vs AtomicReference, final to make sure it is only initialized once, etc.
    Gamepad g1;
    MultipleTelemetry mtelemetry;

    @Override
    public void init() {
        g1 = gamepad1;
        mtelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }
        @Override
    public void init_loop()  {

    }
    public void loop() {
        mtelemetry.addData("cross",g1.cross);
        mtelemetry.addData("circle",g1.circle);
        mtelemetry.addData("square",g1.square);
        mtelemetry.addData("triangle",g1.triangle);
        mtelemetry.addData("ps",g1.ps);
        mtelemetry.addData("back",g1.back);
        mtelemetry.addData("guide",g1.guide);
        mtelemetry.addData("share",g1.share);
        mtelemetry.addData("start",g1.start);
        mtelemetry.addData("options",g1.options);
    }
}