package org.firstinspires.ftc.teamcode.config.seqcommands;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.config.commands.DoPath;
import org.firstinspires.ftc.teamcode.config.commands.Shrink;
import org.firstinspires.ftc.teamcode.config.core.Robot;

public class Fix extends SequentialCommandGroup {
    public Fix(Robot r, boolean keep) {
        addCommands(
            new DoPath(r,true,true, false),
//            new TargetLock(r,true,0, xMin, xMax, refind, keep),
//            new WaitCommand(100),
            new InstantCommand(() -> {
                r.getI().close();
            }),
            new WaitCommand(200),
//            new InstantCommand(() -> {
//                r.getI().hovero(false,r.getE().getPos());
//                r.getI().close();
//            }),
//            new WaitCommand(200),
            new ConditionalCommand(
                new ParallelCommandGroup(
                    new Shrink(r),
                    new InstantCommand(() -> r.getI().throwbackD())
                        .andThen(new WaitCommand(100))
                ),
                new ParallelCommandGroup(
                    new Shrink(r),
                    new InstantCommand(() -> r.getI().transfer())
                        .andThen(new WaitCommand(350))
                ),
            () -> keep)
        );
    }
}