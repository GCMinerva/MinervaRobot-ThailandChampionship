package org.firstinspires.ftc.teamcode.opmode;

import static org.firstinspires.ftc.teamcode.config.core.Robot.autoEndPose;

import com.arcrobotics.ftclib.command.Command;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.config.core.Robot;
import org.firstinspires.ftc.teamcode.config.core.util.Alliance;
import org.firstinspires.ftc.teamcode.config.core.util.OpModeCommand;

import java.util.List;

@TeleOp(name = "Dual Drive")
public class DualDrive extends OpModeCommand {
    Robot r;

    @Override
    public void initialize() {
        r = new Robot(hardwareMap, telemetry, gamepad1, gamepad2, Alliance.BLUE, autoEndPose, true);
    }

    @Override
    public void start() {
        r.tStart();
    }

    @Override
    public void iter() {
        List<Command> commands = r.dualControls();
        schedule(commands.toArray(new Command[0]));
        r.tPeriodic();
    }

    @Override
    public void end() {

    }
}
