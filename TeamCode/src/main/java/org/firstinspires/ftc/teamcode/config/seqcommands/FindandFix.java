package org.firstinspires.ftc.teamcode.config.seqcommands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.config.commands.GenPath;
import org.firstinspires.ftc.teamcode.config.commands.ProcessFrame;
import org.firstinspires.ftc.teamcode.config.core.Robot;
import org.firstinspires.ftc.teamcode.config.core.util.Color;
import org.firstinspires.ftc.teamcode.config.core.util.OptionalCommand;

public class FindandFix extends SequentialCommandGroup {
    public FindandFix(Robot r, Color color, boolean refind, double xMin, double xMax, boolean keep, int need) {
        addCommands(
//            new InstantCommand(() -> r.getO().specimen()),
//            new WaitUntilCommand(() -> r.getF().getCurrentTValue() > 0.99),
            new OptionalCommand(refind, new SequentialCommandGroup(
                new WaitUntilCommand(() -> r.getF().getVelocity().getMagnitude() < 0.5),
                new ProcessFrame(r, color, need),
                new WaitUntilCommand(() -> r.getV().cnt2 > 0)
            )),
//            new OptionalCommand(!refind, new WaitUntilCommand(() -> r.getF().getVelocity().getMagnitude() < 0.2)),
            new GenPath(r,0,xMin,xMax,false),
            new Fix(r,keep)
        );
    }
}