package org.firstinspires.ftc.teamcode.config.core.util;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;

public class OptionalCommand extends ConditionalCommand {
    public OptionalCommand(boolean condition, Command command) {
        super(command, new InstantCommand(() -> {}), () -> condition);
    }
}
