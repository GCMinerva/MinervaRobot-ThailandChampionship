package org.firstinspires.ftc.teamcode.archived.auto.test.commands.actuator;

import com.arcrobotics.ftclib.command.InstantCommand;
import org.firstinspires.ftc.teamcode.archived.subsystems.ActuatorSubsystem;

public class RunIntake extends InstantCommand {
    public RunIntake(ActuatorSubsystem actuator) {
        super(() -> actuator.runIntake());
    }
}
