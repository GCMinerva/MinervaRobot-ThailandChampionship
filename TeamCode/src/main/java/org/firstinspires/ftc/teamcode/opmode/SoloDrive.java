package org.firstinspires.ftc.teamcode.opmode;

import static org.firstinspires.ftc.teamcode.config.core.Robot.autoEndPose;
import static org.firstinspires.ftc.teamcode.config.core.RobotConstants.currentAlliance;

import com.arcrobotics.ftclib.command.Command;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.config.core.Robot;
import org.firstinspires.ftc.teamcode.config.core.paths.CVPaths;
import org.firstinspires.ftc.teamcode.config.core.util.Color;
import org.firstinspires.ftc.teamcode.config.core.util.OpModeCommand;

import java.util.List;

@TeleOp(name = "Solo Drive")
public class SoloDrive extends OpModeCommand {
    Robot r;

    @Override
    public void initialize() {
        r = new Robot(hardwareMap, telemetry, gamepad1, gamepad2, currentAlliance, autoEndPose, false);
        r.initVision(Color.YELLOW);
        CVPaths.paths.clear();
    }

    @Override
    public void start() {
        r.tStart();
    }

    @Override
    public void iter() {
        List<Command> commands = r.soloControls();
        schedule(commands.toArray(new Command[0]));
        r.tPeriodic();
    }

    @Override
    public void end() {

    }
}
