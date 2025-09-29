package org.firstinspires.ftc.teamcode.opmode.auto;

import static org.firstinspires.ftc.teamcode.config.core.RobotConstants.currentAlliance;
import static org.firstinspires.ftc.teamcode.config.core.RobotConstants.liftToInit;
import static org.firstinspires.ftc.teamcode.config.core.paths.FiveSpec.grab1;
import static org.firstinspires.ftc.teamcode.config.core.paths.FiveSpec.grab2;
import static org.firstinspires.ftc.teamcode.config.core.paths.FiveSpec.grab3;
import static org.firstinspires.ftc.teamcode.config.core.paths.FiveSpec.pickup1;
import static org.firstinspires.ftc.teamcode.config.core.paths.FiveSpec.pickup2;
import static org.firstinspires.ftc.teamcode.config.core.paths.FiveSpec.pickup3;
import static org.firstinspires.ftc.teamcode.config.core.paths.FiveSpec.spec1;
import static org.firstinspires.ftc.teamcode.config.core.paths.FiveSpec.spec2;
import static org.firstinspires.ftc.teamcode.config.core.paths.FiveSpec.spec3;
import static org.firstinspires.ftc.teamcode.config.core.paths.FiveSpec.start;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.pedropathing.commands.FollowPath;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.config.commands.ForwardChamber;
import org.firstinspires.ftc.teamcode.config.commands.GenPath;
import org.firstinspires.ftc.teamcode.config.commands.GrabSpec1;
import org.firstinspires.ftc.teamcode.config.commands.GrabSpec2;
import org.firstinspires.ftc.teamcode.config.commands.GrabSpec3;
import org.firstinspires.ftc.teamcode.config.commands.ProcessFrame;
import org.firstinspires.ftc.teamcode.config.core.Robot;
import org.firstinspires.ftc.teamcode.config.core.paths.CVPaths;
import org.firstinspires.ftc.teamcode.config.core.subsystems.Intake;
import org.firstinspires.ftc.teamcode.config.core.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.config.core.util.Alliance;
import org.firstinspires.ftc.teamcode.config.core.util.Color;
import org.firstinspires.ftc.teamcode.config.core.util.OpModeCommand;
import org.firstinspires.ftc.teamcode.config.seqcommands.Fix;

@Autonomous(group = "Auto", name = "6+0")
public class SixSpec extends OpModeCommand {
    Robot r;
    private Timer aTimer;
    private Color color;
    private static boolean found;

    @Override
    public void initialize() {
        r = new Robot(hardwareMap, telemetry, currentAlliance, start);
        if (currentAlliance == Alliance.BLUE) {
            color = Color.BLUE;
        } else if (currentAlliance == Alliance.RED) {
            color = Color.RED;
        }
        r.getI().init();
        r.getO().init();
        r.getL().init();
        r.getE().init();
        r.initVision(color);
        r.getT().addData("init", true);
        r.getT().update();
        CVPaths.paths.clear();

        schedule(
            new RunCommand(r::aPeriodic),
            new SequentialCommandGroup(
                new WaitCommand(0),
                new InstantCommand(() -> r.getL().toSpecimen())
                    .andThen(new WaitUntilCommand(() -> r.getL().percentageToTarget(0.7))),
                new ForwardChamber(r)
                    .alongWith(
                        new FollowPath(r.getF(), spec1()).setCompletionThreshold(0.95)
//                            .alongWith(new WaitUntilCommand(() -> r.getF().getCurrentTValue() > 0.98)
//                                .andThen(new InstantCommand(() -> {
//                                        r.getO().open();
//                                    })
//                                )
//                            )
                    ),
                new FollowPath(r.getF(), grab1()).setCompletionThreshold(0.95).setHoldEnd(true)
                    .alongWith(new WaitCommand(100)
                        .andThen(
                            new WaitUntilCommand(() -> r.getF().getCurrentTValue() > 0.85)
                                .andThen(new ParallelCommandGroup(
                                        new GrabSpec1(r)
                                    )
                                        .alongWith(new WaitCommand(400)
                                            .andThen(new WaitUntilCommand(() -> r.getI().pivotState == Intake.PivotState.THROWBACK)
                                                .andThen(new ParallelCommandGroup(
                                                        new WaitCommand(100)
                                                            .andThen(new FollowPath(r.getF(), grab2()).setCompletionThreshold(0.95).setHoldEnd(true)),
                                                        new WaitCommand(450)
                                                            .andThen(new GrabSpec2(r))
                                                    )
                                                )
                                            )
                                        )
                                )
                        )
                    ),
                new GrabSpec3(r)
                    .alongWith(new FollowPath(r.getF(), grab3()).setCompletionThreshold(0.95).setHoldEnd(true)),
//                new ParallelCommandGroup(
//                    new SequentialCommandGroup(
//                        new GrabSpec2(r),
//                        new GrabSpec3(r)
//                    ),
//                    new SequentialCommandGroup(
//                        new WaitCommand(100)
//                            .andThen(new WaitUntilCommand(() -> r.getI().pivotState == Intake.PivotState.THROWBACK)
//                                .andThen(new WaitCommand(100)
//                                    .andThen(new FollowPath(r.getF(), grab3()).setCompletionThreshold(0.95).setHoldEnd(true))
//                                )
//                            )
//                    )
//                ),
//                new GrabSpec2(r)
//                    .alongWith(new WaitCommand(100)
//                        .andThen(new WaitUntilCommand(() -> r.getI().pivotState == Intake.PivotState.THROWBACK)
//                            .andThen(new WaitCommand(100)
//                                .andThen(new FollowPath(r.getF(), grab3()).setCompletionThreshold(0.95).setHoldEnd(true))
//                            )
//                        )
//                    ),
                new FollowPath(r.getF(), pickup1()).setCompletionThreshold(0.99)
                    .alongWith(new WaitUntilCommand(() -> r.getF().getCurrentTValue() > 0.95)
                        .andThen(new InstantCommand(() -> r.getI().open()))),
                new WaitUntilCommand(() -> r.getF().getVelocityMagnitude() < 0.5),
                new ParallelCommandGroup(
                    new WaitCommand(100)
                        .andThen(new FollowPath(r.getF(), spec2()).setCompletionThreshold(0.95)),
                    new ForwardChamber(r)
                ),
//                new FindandFix(r,color,true,-10,3,true,2),
                new WaitUntilCommand(() -> r.getF().getVelocity().getMagnitude() < 1),
                new ProcessFrame(r, color, 1),
                new WaitUntilCommand(() -> r.getV().cnt2 > 0),
                new GenPath(r,0,-8,3,false),
                new Fix(r,true),
                new ParallelCommandGroup(
                    new FollowPath(r.getF(), pickup3()).setCompletionThreshold(0.99)
                        .alongWith(new ParallelCommandGroup(
                                new WaitUntilCommand(() -> r.getF().getCurrentTValue() > 0.5).
                                    andThen(new InstantCommand(() -> r.getI().throwback())),
                                new WaitUntilCommand(() -> r.getF().getCurrentTValue() > 0.95).
                                    andThen(new InstantCommand(() -> r.getI().open()))
                            )
                        ),
                    new WaitCommand(500).
                        andThen(new InstantCommand(() -> {
                            r.getO().pickup();
                        }))
                ),
                new WaitUntilCommand(() -> r.getF().getVelocityMagnitude() < 0.5),
                new ParallelCommandGroup(
                    new WaitCommand(100)
                        .andThen(new FollowPath(r.getF(), spec2()).setCompletionThreshold(0.95)),
                    new ForwardChamber(r)
//                    new GenPath(r,0,-8,3)
                ),
//                new ConditionalCommand(new FindandFix(r,color,true,-6,3,true,1),
//                new FindandFix(r,color,false,-10,3,true,0),
//                    () -> r.getV().getSamples().isEmpty()),
//                new Fix(r,true),
                new ParallelCommandGroup(
                    new WaitCommand(500).
                        andThen(new InstantCommand(() -> {
                            r.getO().pickup();
                        })),
                    new FollowPath(r.getF(), pickup2()).setCompletionThreshold(0.99)
                ),
                new WaitUntilCommand(() -> r.getF().getVelocityMagnitude() < 0.5),
                new ParallelCommandGroup(
                    new WaitCommand(100)
                        .andThen(new FollowPath(r.getF(), spec3()).setCompletionThreshold(0.95)),
                    new ForwardChamber(r)
                ),
                new ParallelCommandGroup(
                    new WaitCommand(500).
                        andThen(new InstantCommand(() -> {
                            r.getO().pickup();
                        })),
                    new FollowPath(r.getF(), pickup2()).setCompletionThreshold(0.99)
                ),
                new WaitUntilCommand(() -> r.getF().getVelocityMagnitude() < 0.5),
                new ParallelCommandGroup(
                    new WaitCommand(100)
                        .andThen(new FollowPath(r.getF(), spec3()).setCompletionThreshold(0.95)),
                    new ForwardChamber(r)
                ),
                new ParallelCommandGroup(
                    new WaitCommand(500).
                        andThen(new InstantCommand(() -> {
                            r.getO().pickup();
                        })),
                    new FollowPath(r.getF(), pickup2()).setCompletionThreshold(0.95)
                ),
                new WaitUntilCommand(() -> r.getF().getVelocityMagnitude() < 0.5),
                new ParallelCommandGroup(
                    new WaitCommand(100)
                        .andThen(new FollowPath(r.getF(), spec3()).setCompletionThreshold(0.95)),
                    new ForwardChamber(r)
                ),
                new ParallelCommandGroup(
                    new WaitCommand(500).
                        andThen(new InstantCommand(() -> {
                            r.getO().pickup();
                        })),
                    new FollowPath(r.getF(), pickup2()).setCompletionThreshold(0.95)
                )
            )
        );

        aTimer = new Timer();
        r.getO().open();
    }

    @Override
    public void init_loop() {
        super.init_loop();
        if (aTimer.getElapsedTimeSeconds() > 1 && r.getL().getPos() != liftToInit) {
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