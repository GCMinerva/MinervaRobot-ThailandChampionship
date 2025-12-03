package org.firstinspires.ftc.teamcode.archived.auto.test.opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.archived.subsystems.ActuatorSubsystem;
import org.firstinspires.ftc.teamcode.archived.subsystems.AprilTagSubsystem;
import org.firstinspires.ftc.teamcode.archived.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.archived.auto.test.commands.AutoBallDetection;
import org.firstinspires.ftc.teamcode.archived.auto.test.commands.FollowPath;
import org.firstinspires.ftc.teamcode.archived.auto.test.commands.WaitUntilPathPercent;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.archived.util.GlobalData;
import org.firstinspires.ftc.teamcode.archived.auto.test.commands.OpModeCommand;

@Autonomous(name = "RedFar[KAO] - Only drive", group = "Auton")
public class RED2kao extends OpModeCommand {

    private Follower follower;
    private AprilTagSubsystem tagSubsystem;
    private ActuatorSubsystem actuator;
    private TurretSubsystem turret;
    // Starting pose
    private final Pose startPose = new Pose(87, 9.5, Math.toRadians(90));

    @Override
    public void initialize() {
        // === Initialize Subsystems ===
        tagSubsystem = new AprilTagSubsystem(hardwareMap, telemetry);
        actuator = new ActuatorSubsystem(hardwareMap, telemetry);
        turret = new TurretSubsystem(hardwareMap);
        follower = Constants.createFollower(hardwareMap);

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

        // Schedule follower to follow paths sequentially
        schedule(
                new SequentialCommandGroup(
                        new FollowPath(follower, path1)
                                .setMaxPower(1)
                                .alongWith(
                                        new SequentialCommandGroup(
                                                new WaitCommand(20)
                                                        .andThen(new InstantCommand(
                                                                () -> actuator.runOuttake(-0.9))),

                                                new WaitCommand(60) // 1400
                                                        .andThen(new InstantCommand(
                                                                () -> turret.turnToAngle(
                                                                        0)))
                                                // At 1% of path, start
                                                // shooter to spin up
                                        ))
                                // After path completes, hold current position
                                .andThen(new InstantCommand(() -> follower
                                        .holdPoint(follower.getPose()))),
                                // Then shoot while stationary
                                new WaitCommand(650).andThen(new InstantCommand(
                                        () -> actuator.justShootFAR())),
                        new InstantCommand(() -> follower.breakFollowing()),
                        new WaitCommand(200),

                        new FollowPath(follower, path2).setMaxPower(1.0)
                                .alongWith(
                                        new SequentialCommandGroup(
                                                // At 1% stop shooter
                                                new WaitUntilPathPercent(
                                                        follower,
                                                        0.01)
                                                        .andThen(new InstantCommand(
                                                                () -> actuator.stopShooter())),
                                                // At 60% run intake
                                                new WaitUntilPathPercent(
                                                        follower,
                                                        0.40)
                                                        .andThen(new InstantCommand(
                                                                () -> actuator.runIntake())),
                                                // At 90% sorter IN1
                                                new WaitUntilPathPercent(
                                                        follower,
                                                        0.40)
                                                        .andThen(new InstantCommand(
                                                                () -> actuator.setSorterIN1())))),

                        new FollowPath(follower, path3).setMaxPower(0.3) // 21
                                .alongWith(
                                        new SequentialCommandGroup(
                                                // Start automatic ball
                                                // detection
                                                new InstantCommand(
                                                        () -> actuator.initBallDetection()),
                                                // Start shooter after
                                                // brief delay
                                                new WaitCommand(10)
                                                        .andThen(new InstantCommand(
                                                                () -> actuator.runShooterClose())),
                                                // Run automatic ball
                                                // detection until 3
                                                // balls collected
                                                new AutoBallDetection(
                                                        actuator))),
                        new WaitCommand(100),

                        new FollowPath(follower, path4).setMaxPower(1.0)
                                .alongWith(
                                        new SequentialCommandGroup(
                                                // At 10% stop intake
                                                new InstantCommand(
                                                        () -> actuator.FarGOready()),
                                                new WaitUntilPathPercent(
                                                        follower,
                                                        0.90)
                                                        .andThen(new InstantCommand(
                                                                () -> actuator.stopIntake()))

                                        ))
                                // // After path completes, hold current position
                                .andThen(new InstantCommand(() -> follower
                                        .holdPoint(follower.getPose()))),
                        // // Then shoot while stationary
                        new WaitCommand(100)
                                .andThen(new InstantCommand(
                                        () -> actuator.justShootFAR())),

                        //path 5 -- run intake
                        new FollowPath(follower, path5).setMaxPower(1.0)
                                .alongWith(
                                        new SequentialCommandGroup(
                                                // At 1% stop shooter
                                                new WaitUntilPathPercent(
                                                        follower,
                                                        0.01)
                                                        .andThen(new InstantCommand(
                                                                () -> actuator.stopShooter())),
                                                // At 60% run intake
                                                new WaitUntilPathPercent(
                                                        follower,
                                                        0.40)
                                                        .andThen(new InstantCommand(
                                                                () -> actuator.runIntake())),
                                                // At 90% sorter IN1
                                                new WaitUntilPathPercent(
                                                        follower,
                                                        0.40)
                                                        .andThen(new InstantCommand(
                                                                () -> actuator.setSorterIN1())))),

                        new FollowPath(follower, path6).setMaxPower(0.26) // 21
                                .alongWith(
                                        new SequentialCommandGroup(
                                                // Start automatic ball
                                                // detection
                                                new InstantCommand(
                                                        () -> actuator.initBallDetection()),
                                                // Start shooter after
                                                // brief delay
                                                new WaitCommand(10)
                                                        .andThen(new InstantCommand(
                                                                () -> actuator.runShooterClose())),
                                                // Run automatic ball
                                                // detection until 3
                                                // balls collected
                                                new AutoBallDetection(
                                                        actuator))),
                        new WaitCommand(100),

                        new FollowPath(follower, path7).setMaxPower(1.0)
                                .alongWith(
                                        new SequentialCommandGroup(
                                                // At 10% stop intake
                                                new InstantCommand(
                                                        () -> actuator.FarGOready()),
                                                new WaitUntilPathPercent(
                                                        follower,
                                                        0.90)
                                                        .andThen(new InstantCommand(
                                                                () -> actuator.stopIntake()))

                                        ))
                                // // After path completes, hold current position
                                .andThen(new InstantCommand(() -> follower
                                        .holdPoint(follower.getPose()))),
                        // // Then shoot while stationary
                        new WaitCommand(100)
                                .andThen(new InstantCommand(
                                        () -> actuator.justShoot())),

                        //path 8 -- run intake
                        new FollowPath(follower, path8).setMaxPower(1.0)
                                .alongWith(
                                        new SequentialCommandGroup(
                                                // At 1% stop shooter
                                                new WaitUntilPathPercent(
                                                        follower,
                                                        0.01)
                                                        .andThen(new InstantCommand(
                                                                () -> actuator.stopShooter())),
                                                // At 60% run intake
                                                new WaitUntilPathPercent(
                                                        follower,
                                                        0.40)
                                                        .andThen(new InstantCommand(
                                                                () -> actuator.runIntake())),
                                                // At 90% sorter IN1
                                                new WaitUntilPathPercent(
                                                        follower,
                                                        0.40)
                                                        .andThen(new InstantCommand(
                                                                () -> actuator.setSorterIN1())))),

                        new FollowPath(follower, path9).setMaxPower(0.3) // 21
                                .alongWith(
                                        new SequentialCommandGroup(
                                                // Start automatic ball
                                                // detection
                                                new InstantCommand(
                                                        () -> actuator.initBallDetection()),
                                                // Start shooter after
                                                // brief delay
                                                new WaitCommand(10)
                                                        .andThen(new InstantCommand(
                                                                () -> actuator.runShooterClose())),
                                                // Run automatic ball
                                                // detection until 3
                                                // balls collected
                                                new AutoBallDetection(
                                                        actuator))),
                        new WaitCommand(100),

                        new FollowPath(follower, path10).setMaxPower(1.0)
                                .alongWith(
                                        new SequentialCommandGroup(
                                                // At 10% stop intake
                                                new InstantCommand(
                                                        () -> actuator.FarGOready()),
                                                new WaitUntilPathPercent(
                                                        follower,
                                                        0.90)
                                                        .andThen(new InstantCommand(
                                                                () -> actuator.stopIntake()))

                                        ))
                                // // After path completes, hold current position
                                .andThen(new InstantCommand(() -> follower
                                        .holdPoint(follower.getPose()))),
                        // // Then shoot while stationary
                        new WaitCommand(100)
                                .andThen(new InstantCommand(
                                        () -> actuator.justShoot())),

                        new InstantCommand(() -> follower.breakFollowing())
                )
        );
    }

    @Override
    public void iter() {
        // Nothing else to do - only driving
    }

    @Override
    public void end() {
        // Save final pose if needed
        Pose finalPose = follower.getPose();
        if (finalPose != null) {
            // Optional: store final pose
        }
    }

    // Path builders (same as original)
    // ---------------- PATH BUILDERS ----------------
    private PathChain buildPath1() {
        return new PathBuilder(follower, Constants.pathConstraints)
                .addPath(new BezierLine(new Pose(87.000, 9.500), new Pose(87.010, 14.306)))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(67))
                .build();
    }

    private PathChain buildPath2() {
        return new PathBuilder(follower, Constants.pathConstraints)
                .addPath(new BezierCurve(new Pose(87.010, 14.306),
                        new Pose(86.306, 31.192),
                        new Pose(100.612, 40.104)))
                .setLinearHeadingInterpolation(Math.toRadians(67), Math.toRadians(0))
                .build();
    }

    private PathChain buildPath3() {
        return new PathBuilder(follower, Constants.pathConstraints)
                .addPath(new BezierLine(new Pose(100.612, 40.104), new Pose(130, 40.339)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
    }

    private PathChain buildPath4() {
        return new PathBuilder(follower, Constants.pathConstraints)
                .addPath(new BezierLine(new Pose(130, 40.339), new Pose(83.257, 17.121)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(67))
                .build();
    }

    private PathChain buildPath5() {
        return new PathBuilder(follower, Constants.pathConstraints)
                .addPath(new BezierCurve(new Pose(83.257, 17.121),
                        new Pose(83.257, 54.645),
                        new Pose(101.081, 64.730)))
                .setLinearHeadingInterpolation(Math.toRadians(67), Math.toRadians(0))
                .build();
    }

    private PathChain buildPath6() {
        return new PathBuilder(follower, Constants.pathConstraints)
                .addPath(new BezierLine(new Pose(101.081, 64.730), new Pose(125.238, 64.261)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
    }

    private PathChain buildPath7() {
        return new PathBuilder(follower, Constants.pathConstraints)
                .addPath(new BezierCurve(new Pose(125.238, 64.261),
                        new Pose(85.603, 68.717),
                        new Pose(87.948, 87.479)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                .build();
    }

    private PathChain buildPath8() {
        return new PathBuilder(follower, Constants.pathConstraints)
                .addPath(new BezierCurve(new Pose(87.948, 87.479),
                        new Pose(100.847, 87.948)))
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                .build();
    }

    private PathChain buildPath9() {
        return new PathBuilder(follower, Constants.pathConstraints)
                .addPath(new BezierLine(new Pose(100.847, 87.948), new Pose(125.238, 88.417)))
                .setTangentHeadingInterpolation()
                .build();
    }

    private PathChain buildPath10() {
        return new PathBuilder(follower, Constants.pathConstraints)
                .addPath(new BezierLine(new Pose(125.238, 88.417), new Pose(88.182, 87.479)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                .build();
    }
}
