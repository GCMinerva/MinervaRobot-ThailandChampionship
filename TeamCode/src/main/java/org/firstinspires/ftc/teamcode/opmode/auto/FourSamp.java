package org.firstinspires.ftc.teamcode.opmode.auto;

import static org.firstinspires.ftc.teamcode.config.core.RobotConstants.liftToInit;
import static org.firstinspires.ftc.teamcode.config.core.RobotConstants.liftToTransfer;
import static org.firstinspires.ftc.teamcode.config.core.paths.FourSamp.grab3;
import static org.firstinspires.ftc.teamcode.config.core.paths.FourSamp.grab4;
import static org.firstinspires.ftc.teamcode.config.core.paths.FourSamp.score1;
import static org.firstinspires.ftc.teamcode.config.core.paths.FourSamp.score2;
import static org.firstinspires.ftc.teamcode.config.core.paths.FourSamp.score3;
import static org.firstinspires.ftc.teamcode.config.core.paths.FourSamp.start;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.pedropathing.commands.FollowPath;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.config.commands.AutoTransport;
import org.firstinspires.ftc.teamcode.config.commands.Bucket1;
import org.firstinspires.ftc.teamcode.config.commands.GrabSamp1;
import org.firstinspires.ftc.teamcode.config.core.Robot;
import org.firstinspires.ftc.teamcode.config.core.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.config.core.util.Alliance;
import org.firstinspires.ftc.teamcode.config.core.util.OpModeCommand;

@Autonomous(group = "Auto", name = "0+4")
public class FourSamp extends OpModeCommand {
    Robot r;
    private Timer aTimer;

    @Override
    public void initialize() {
        r = new Robot(hardwareMap, telemetry, Alliance.BLUE, start);
        r.getI().init();
        r.getO().init();
        r.getL().init();
        r.getE().init();
        r.getT().addData("init", true);
        r.getT().update();

        schedule(
            new RunCommand(r::aPeriodic),
            new SequentialCommandGroup(
                new WaitCommand(0),
                new Bucket1(r)
                    .alongWith(new FollowPath(r.getF(), score1()).setCompletionThreshold(0.99)
                        .alongWith(new GrabSamp1(r))
                    ),
                new AutoTransport(r,1,8.5)
                    .alongWith(new WaitCommand(100)
                        .andThen(new WaitUntilCommand(() -> r.getL().getTarget() != liftToTransfer)
                            .andThen(new FollowPath(r.getF(), score2()).setCompletionThreshold(0.95))
                        )
                    ),
                new AutoTransport(r,2,-18),
                new FollowPath(r.getF(), grab3()).setCompletionThreshold(0.95).setHoldEnd(true)
                    .andThen(new WaitUntilCommand(() -> r.getF().getCurrentTValue() > 0.99)),
                new WaitCommand(0)
                    .andThen(new InstantCommand(() -> {
                        r.getI().groundoZ(true,r.getE().getPos());
                        r.getI().rotateDegrees(-18);
                    }).andThen(new WaitCommand(200)
                        .andThen(new InstantCommand(() -> r.getI().close())
                            .andThen(new WaitCommand(150))
                        )
                        )
                    ),
                new AutoTransport(r,0,0)
                    .alongWith(new WaitCommand(100)
                        .andThen(new WaitUntilCommand(() -> r.getL().getTarget() != liftToTransfer)
                            .andThen(new FollowPath(r.getF(), score3()).setCompletionThreshold(0.95))
                        )
                    ),
                new WaitCommand(150),
                new InstantCommand(() -> {
                    r.getL().toSpecimen();
                    r.getO().specimen();
                }),
                new FollowPath(r.getF(), grab4()).setCompletionThreshold(0.95)
            )
        );

        aTimer = new Timer();
        r.getO().open();
    }

    @Override
    public void init_loop() {
        super.init_loop();
        if (aTimer.getElapsedTimeSeconds() > 1 && Math.abs(r.getL().getPos()-liftToInit) > 50) {
            r.getL().setTarget(liftToInit);
            r.getL().periodic();
        }
        if (aTimer.getElapsedTimeSeconds() > 3 && r.getO().grabState != Outtake.GrabState.CLOSED) {
            r.getO().close();
        }
    }

    @Override
    public void iter() {

    }
    
    @Override
    public void end() {
        Robot.autoEndPose = r.getF().getPose();
    }
}