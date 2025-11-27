package org.firstinspires.ftc.teamcode.archived.auto.test.opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.archived.subsystems.ActuatorSubsystem;
import org.firstinspires.ftc.teamcode.archived.subsystems.AprilTagSubsystem;
import org.firstinspires.ftc.teamcode.archived.auto.test.commands.FollowPath;
import org.firstinspires.ftc.teamcode.archived.auto.test.commands.WaitUntilPathPercent;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.GlobalData;
import org.firstinspires.ftc.teamcode.archived.auto.test.commands.OpModeCommand;

@Autonomous(name = "AutoDawnCommand", group = "AprilTest")
public class AutoDawnCommand extends OpModeCommand {

    private Follower follower;
    private AprilTagSubsystem tagSubsystem;
    private ActuatorSubsystem actuator;
    
    // Starting pose
    private final Pose startPose = new Pose(110.5, 129.5, Math.toRadians(90));

    @Override
    public void initialize() {
        // === Initialize Subsystems ===
        tagSubsystem = new AprilTagSubsystem(hardwareMap, telemetry);
        actuator = new ActuatorSubsystem(hardwareMap, telemetry);
        follower = Constants.createFollower(hardwareMap);
        
        telemetry.addLine("🔍 Scanning for AprilTags...");
        telemetry.addLine("⚙️ Actuator ready");
        telemetry.update();
        
        // Open bin during init
        actuator.openBin();
        
        // Set starting pose
        follower.setStartingPose(startPose);
        
        // Build all paths
        PathChain path1 = buildPath1();
        PathChain path2 = buildPath2();
        PathChain path3 = buildPath3();
        PathChain path4 = buildPath4();
        PathChain path5 = buildPath5();
        PathChain path6 = buildPath6();
        PathChain path7 = buildPath7();
        PathChain path8 = buildPath8();
        PathChain path9 = buildPath9();
        PathChain path10 = buildPath10();
        PathChain path11 = buildPath11();

        // Schedule command sequence
        schedule(
                // Telemetry display
                new RunCommand(() -> {
                    Pose p = follower.getPose();
                    telemetry.addData("Pose", "X: %.1f, Y: %.1f, H: %.1f°", 
                        p.getX(), p.getY(), Math.toDegrees(p.getHeading()));
                    telemetry.addData("Follower Busy", follower.isBusy());
                    telemetry.addData("Path Progress", "%.1f%%", follower.getCurrentTValue() * 100);
                    telemetry.update();
                }),
                
                // Main autonomous sequence
                new SequentialCommandGroup(
                        // Wait for start
                        new WaitCommand(100),
                        
                        // Path 1: Initial scoring with percentage-based triggers
                        new FollowPath(follower, path1).setMaxPower(1.0)
                                .alongWith(
                                        new SequentialCommandGroup(
                                                // At 12% of path, start shooter
                                                new WaitUntilPathPercent(follower, 0.12)
                                                        .andThen(new InstantCommand(() -> actuator.runShooterClose())),
                                                // At 95% of path, shoot
                                                new WaitUntilPathPercent(follower, 0.95)
                                                        .andThen(new InstantCommand(() -> actuator.justShoot()))
                                        )
                                )
                                .andThen(new WaitCommand(300)),
                        
                        // Path 2: First sample pickup
                        new FollowPath(follower, path2).setMaxPower(1.0)
                                .alongWith(
                                        new SequentialCommandGroup(
                                                // At 1% stop shooter
                                                new WaitUntilPathPercent(follower, 0.01)
                                                        .andThen(new InstantCommand(() -> actuator.stopShooter())),
                                                // At 60% run intake
                                                new WaitUntilPathPercent(follower, 0.60)
                                                        .andThen(new InstantCommand(() -> actuator.runIntake())),
                                                // At 90% sorter IN1
                                                new WaitUntilPathPercent(follower, 0.90)
                                                        .andThen(new InstantCommand(() -> actuator.setSorterIN1()))
                                        )
                                ),
                        
                        // Path 3: Continue to sample
                        new FollowPath(follower, path3).setMaxPower(0.25)
                                .alongWith(
                                        new SequentialCommandGroup(
                                                // At 35% sorter IN2
                                                new WaitUntilPathPercent(follower, 0.35)
                                                        .andThen(new InstantCommand(() -> actuator.setSorterIN2())),
                                                // At 60% sorter IN3
                                                new WaitUntilPathPercent(follower, 0.60)
                                                        .andThen(new InstantCommand(() -> actuator.setSorterIN3()))
                                        )
                                ),
                        
                        // Path 4: Return to shooting position
                        new FollowPath(follower, path4).setMaxPower(0.7)
                                .alongWith(
                                        new SequentialCommandGroup(
                                                // At 10% stop intake
                                                new WaitUntilPathPercent(follower, 0.10)
                                                        .andThen(new InstantCommand(() -> actuator.stopIntake())),
                                                // At 12% start shooter
                                                new WaitUntilPathPercent(follower, 0.12)
                                                        .andThen(new InstantCommand(() -> actuator.runShooterClose()))
                                        )
                                ),
                        
                        // Path 5: Shoot first sample
                        new FollowPath(follower, path5).setMaxPower(1.0)
                                .alongWith(
                                        new SequentialCommandGroup(
                                                // At 100% shoot
                                                new WaitUntilPathPercent(follower, 1.00)
                                                        .andThen(new InstantCommand(() -> actuator.FarGO()))
                                        )
                                )
                                .andThen(new WaitCommand(300)),
                        
                        // Path 6: Second sample pickup
                        new FollowPath(follower, path6).setMaxPower(1.0)
                                .alongWith(
                                        new SequentialCommandGroup(
                                                // At 1% stop shooter
                                                new WaitUntilPathPercent(follower, 0.01)
                                                        .andThen(new InstantCommand(() -> actuator.stopShooter())),
                                                // At 60% run intake
                                                new WaitUntilPathPercent(follower, 0.60)
                                                        .andThen(new InstantCommand(() -> actuator.runIntake())),
                                                // At 90% sorter IN1
                                                new WaitUntilPathPercent(follower, 0.90)
                                                        .andThen(new InstantCommand(() -> actuator.setSorterIN1()))
                                        )
                                ),
                        
                        // Path 7: Continue to second sample
                        new FollowPath(follower, path7).setMaxPower(0.25)
                                .alongWith(
                                        new SequentialCommandGroup(
                                                // At 35% sorter IN2
                                                new WaitUntilPathPercent(follower, 0.35)
                                                        .andThen(new InstantCommand(() -> actuator.setSorterIN2())),
                                                // At 60% sorter IN3
                                                new WaitUntilPathPercent(follower, 0.60)
                                                        .andThen(new InstantCommand(() -> actuator.setSorterIN3())),
                                                // At 95% stop intake
                                                new WaitUntilPathPercent(follower, 0.95)
                                                        .andThen(new InstantCommand(() -> actuator.stopIntake())),
                                                // At 100% start shooter
                                                new WaitUntilPathPercent(follower, 1.00)
                                                        .andThen(new InstantCommand(() -> actuator.runShooterClose()))
                                        )
                                ),
                        
                        // Path 8: Shoot second sample
                        new FollowPath(follower, path8).setMaxPower(1.0)
                                .alongWith(
                                        new SequentialCommandGroup(
                                                // At 100% shoot
                                                new WaitUntilPathPercent(follower, 1.00)
                                                        .andThen(new InstantCommand(() -> actuator.MidGO()))
                                        )
                                )
                                .andThen(new WaitCommand(300)),
                        
                        // Path 9: Third sample pickup
                        new FollowPath(follower, path9).setMaxPower(1.0)
                                .alongWith(
                                        new SequentialCommandGroup(
                                                // At 1% stop shooter
                                                new WaitUntilPathPercent(follower, 0.01)
                                                        .andThen(new InstantCommand(() -> actuator.stopShooter())),
                                                // At 60% run intake
                                                new WaitUntilPathPercent(follower, 0.60)
                                                        .andThen(new InstantCommand(() -> actuator.runIntake())),
                                                // At 90% sorter IN1
                                                new WaitUntilPathPercent(follower, 0.90)
                                                        .andThen(new InstantCommand(() -> actuator.setSorterIN1()))
                                        )
                                ),
                        
                        // Path 10: Continue to third sample
                        new FollowPath(follower, path10).setMaxPower(0.25)
                                .alongWith(
                                        new SequentialCommandGroup(
                                                // At 35% sorter IN2
                                                new WaitUntilPathPercent(follower, 0.35)
                                                        .andThen(new InstantCommand(() -> actuator.setSorterIN2())),
                                                // At 60% sorter IN3
                                                new WaitUntilPathPercent(follower, 0.60)
                                                        .andThen(new InstantCommand(() -> actuator.setSorterIN3())),
                                                // At 95% stop intake
                                                new WaitUntilPathPercent(follower, 0.95)
                                                        .andThen(new InstantCommand(() -> actuator.stopIntake())),
                                                // At 100% start shooter
                                                new WaitUntilPathPercent(follower, 1.00)
                                                        .andThen(new InstantCommand(() -> actuator.runShooterClose()))
                                        )
                                ),
                        
                        // Path 11: Final shot
                        new FollowPath(follower, path11).setMaxPower(1.0)
                                .alongWith(
                                        new SequentialCommandGroup(
                                                // At 100% shoot
                                                new WaitUntilPathPercent(follower, 1.00)
                                                        .andThen(new InstantCommand(() -> actuator.NearGO()))
                                        )
                                )
                                .andThen(new WaitCommand(300))
                )
        );
    }

    @Override
    public void init_loop() {
        super.init_loop();
        // Scan for AprilTags during init
        tagSubsystem.update();
        telemetry.addData("Status", "Scanning for AprilTags...");
        telemetry.addData("Detected Tag", GlobalData.detectedTagId);
        telemetry.update();
    }

    @Override
    public void iter() {
        // This is called every loop - telemetry is handled by the RunCommand
    }

    @Override
    public void end() {
        if (tagSubsystem != null) {
            tagSubsystem.stop();
        }
        if (actuator != null) {
            actuator.stopShooter();
            actuator.stopIntake();
        }
    }

    // Path builders
    private PathChain buildPath1() {
        return new PathBuilder(follower, Constants.pathConstraints)
                .addPath(new BezierLine(
                        new Pose(110.5, 129.5),
                        new Pose(87.995, 85.144)))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(48))
                .build();
    }

    private PathChain buildPath2() {
        return new PathBuilder(follower, Constants.pathConstraints)
                .addPath(new BezierCurve(
                        new Pose(87.995, 85.144),
                        new Pose(92.923, 84.321),
                        new Pose(102.512, 83.335)))
                .setLinearHeadingInterpolation(Math.toRadians(48), Math.toRadians(0))
                .build();
    }

    private PathChain buildPath3() {
        return new PathBuilder(follower, Constants.pathConstraints)
                .addPath(new BezierLine(
                        new Pose(102.512, 83.335),
                        new Pose(120.971, 83.246)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
    }

    private PathChain buildPath4() {
        return new PathBuilder(follower, Constants.pathConstraints)
                .addPath(new BezierCurve(
                        new Pose(120.971, 83.246),
                        new Pose(121.361, 77.913),
                        new Pose(122.381, 76.282),
                        new Pose(127.602, 75.000)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                .build();
    }

    private PathChain buildPath5() {
        return new PathBuilder(follower, Constants.pathConstraints)
                .addPath(new BezierLine(
                        new Pose(127.602, 75.000),
                        new Pose(87.995, 85.144)))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(48))
                .build();
    }

    private PathChain buildPath6() {
        return new PathBuilder(follower, Constants.pathConstraints)
                .addPath(new BezierCurve(
                        new Pose(87.995, 85.144),
                        new Pose(88.174, 59.948),
                        new Pose(102.332, 59.320)))
                .setLinearHeadingInterpolation(Math.toRadians(48), Math.toRadians(0))
                .build();
    }

    private PathChain buildPath7() {
        return new PathBuilder(follower, Constants.pathConstraints)
                .addPath(new BezierLine(
                        new Pose(102.332, 59.320),
                        new Pose(120.971, 59.231)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
    }

    private PathChain buildPath8() {
        return new PathBuilder(follower, Constants.pathConstraints)
                .addPath(new BezierLine(
                        new Pose(120.971, 59.231),
                        new Pose(87.995, 85.144)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(48))
                .build();
    }

    private PathChain buildPath9() {
        return new PathBuilder(follower, Constants.pathConstraints)
                .addPath(new BezierCurve(
                        new Pose(87.995, 85.144),
                        new Pose(75.450, 44.714),
                        new Pose(101.795, 35.216)))
                .setLinearHeadingInterpolation(Math.toRadians(48), Math.toRadians(0))
                .build();
    }

    private PathChain buildPath10() {
        return new PathBuilder(follower, Constants.pathConstraints)
                .addPath(new BezierLine(
                        new Pose(101.795, 35.216),
                        new Pose(120.254, 35.306)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
    }

    private PathChain buildPath11() {
        return new PathBuilder(follower, Constants.pathConstraints)
                .addPath(new BezierLine(
                        new Pose(120.254, 35.306),
                        new Pose(87.995, 85.144)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(48))
                .build();
    }
}
