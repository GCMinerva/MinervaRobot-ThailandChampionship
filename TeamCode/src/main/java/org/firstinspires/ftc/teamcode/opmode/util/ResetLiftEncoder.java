package org.firstinspires.ftc.teamcode.opmode.util;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.config.core.subsystems.Lift;

@Config
@TeleOp(group = "Util", name = "Reset Lift Encoder")
public class ResetLiftEncoder extends OpMode {

    Lift l;

    @Override
    public void init() {
        l = new Lift(hardwareMap, telemetry);
        telemetry.addLine("start to reset");
        telemetry.update();
    }

    @Override
    public void loop() {
        l.leftLiftEncoder.reset();
    }
}
