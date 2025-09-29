package org.firstinspires.ftc.teamcode.opmode.util;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.config.core.RobotConstants;
import org.firstinspires.ftc.teamcode.config.core.util.Alliance;

@Config
@TeleOp(group = "Util", name = "Set Alliance")
public class SetAlliance extends OpMode {

    MultipleTelemetry mtelemetry;

    @Override
    public void init() {
        mtelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addLine("x blue b red");
        telemetry.update();
    }

    @Override
    public void loop() {
        if (gamepad1.x) {
            RobotConstants.currentAlliance = Alliance.BLUE;
        } else if (gamepad1.b) {
            RobotConstants.currentAlliance = Alliance.RED;
        }
        if (RobotConstants.currentAlliance == Alliance.BLUE) {
            mtelemetry.addLine("BLUE");
        } else if (RobotConstants.currentAlliance == Alliance.RED) {
            mtelemetry.addLine("RED");
        }
        mtelemetry.update();
    }
}
