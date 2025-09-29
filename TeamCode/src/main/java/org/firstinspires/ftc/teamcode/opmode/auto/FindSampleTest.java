package org.firstinspires.ftc.teamcode.opmode.auto;

import static org.firstinspires.ftc.teamcode.config.core.paths.FiveSpec.start;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.config.core.Robot;
import org.firstinspires.ftc.teamcode.config.core.util.Alliance;
import org.firstinspires.ftc.teamcode.config.core.util.OpModeCommand;
import org.firstinspires.ftc.teamcode.config.seqcommands.FindandFix;
import org.firstinspires.ftc.teamcode.opmode.tests.CVTest;

import java.util.concurrent.ThreadLocalRandom;

@Autonomous(group = "Auto", name = "Find Sample Test")
public class FindSampleTest extends OpModeCommand {
    Robot r;

    @Override
    public void initialize() {
        r = new Robot(hardwareMap, telemetry, Alliance.BLUE, new Pose(ThreadLocalRandom.current().nextInt(-20,20),
            ThreadLocalRandom.current().nextInt(-20,20),
            ThreadLocalRandom.current().nextDouble(-Math.PI,Math.PI)));
        r.getI().init();
        r.getO().init();
//        r.getL().init();
        r.getE().init();
        r.getT().addData("init", true);
        r.getT().update();
        r.initVision(CVTest.color);
        r.getF().setPose(start);
        r.getO().open();

        schedule(
            new RunCommand(r::aPeriodic),
            new SequentialCommandGroup(
                new WaitCommand(0),
//                new ProcessFrame(r, Color.RED),
////                new WaitCommand(3000),
//                new InstantCommand(() -> {
//                    r.getO().score();
//                }),
//                new TargetLock(r,true,0),
//                new InstantCommand(() -> {
//                    r.getI().close();
//                }),
                new FindandFix(r,CVTest.color,true,-20,20,true,1),
                new InstantCommand(() -> r.getI().throwback()),
                new WaitCommand(300),
                new InstantCommand(() -> r.getI().open())
//                new InstantCommand(() -> r.getI().throwback()),
//                new WaitCommand(50),
//                new InstantCommand(() -> r.getI().open()),
//                new FindandFix(r,Color.YELLOW,true),
//                new InstantCommand(() -> r.getI().throwback()),
//                new WaitCommand(50),
//                new InstantCommand(() -> r.getI().open()),
//                new FindandFix(r,Color.YELLOW,true),
//                new InstantCommand(() -> r.getI().throwback()),
//                new WaitCommand(50),
//                new InstantCommand(() -> r.getI().open()),
//                new FindandFix(r,Color.YELLOW,true),
//                new InstantCommand(() -> r.getI().throwback()),
//                new WaitCommand(50),
//                new InstantCommand(() -> r.getI().open())
            )
        );
    }

    @Override
    public void init_loop() {
        super.init_loop();
    }

    @Override
    public void iter() {

    }

    @Override
    public void end() {

    }
}