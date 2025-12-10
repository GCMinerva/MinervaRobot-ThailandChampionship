package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelRaceGroup;
import com.seattlesolvers.solverslib.command.RunCommand;
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

@Disabled
@Autonomous(name = "BlueFar[12Art-Tracking]", group = "Auton")
public class BlueFar12ArtTracking extends OpModeCommand {

        private Follower follower;
        private AprilTagSubsystem tagSubsystem;
        private ActuatorSubsystem actuator;
        private TurretSubsystem turret;

        // Starting pose (mirrored for Blue alliance)
        private final Pose startPose = new Pose(107.5, 126.5, Math.toRadians(90)).mirror();

        // Blue goal position (field coordinates)
        private static final double GOAL_X = 0.0;
        private static final double GOAL_Y = 144.0;

        @Override
        public void initialize() {
                // === Initialize Subsystems ===
                tagSubsystem = new AprilTagSubsystem(hardwareMap, telemetry);
                actuator = new ActuatorSubsystem(hardwareMap, telemetry);
                turret = new TurretSubsystem(hardwareMap);
                follower = Constants.createFollower(hardwareMap);

                // Set turret goal position (Blue alliance goal)
                turret.setGoalPosition(GOAL_X, GOAL_Y);

                // Pre-position turret to -90° (left side) before autonomous starts
                // This is a BLOCKING call, but it's OK here since commands haven't started yet
                telemetry.addLine("⏳ Pre-positioning turret to -90°...");
                telemetry.update();
                turret.turnToAngle(-90);
                telemetry.addLine("✅ Turret ready at -90°");
                telemetry.update();

                telemetry.addLine("🎯 Turret Tracking Enabled");
                telemetry.addData("Goal Position", "(" + GOAL_X + ", " + GOAL_Y + ")");
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
                PathChain path12 = buildPath12();

                // Schedule command sequence with CONTINUOUS TURRET TRACKING
                schedule(
                                new ParallelRaceGroup(
                                                // Main autonomous sequence
                                                new SequentialCommandGroup(
                                                                // Wait for start
                                                                new WaitCommand(10),
                                                                // Path 1: Initial scoring
                                                                new FollowPath(follower, path1)
                                                                                .setMaxPower(1)
                                                                                .alongWith(
                                                                                                new SequentialCommandGroup(
                                                                                                                // Start
                                                                                                                // outtake
                                                                                                                new WaitCommand(20)
                                                                                                                                .andThen(new InstantCommand(
                                                                                                                                                () -> actuator.runOuttake(
                                                                                                                                                                -0.81))),

                                                                                                                // Start
                                                                                                                // shooter
                                                                                                                new WaitCommand(200)
                                                                                                                                .andThen(new InstantCommand(
                                                                                                                                                () -> actuator.runShooterClose()))))
                                                                                // After path completes, hold current
                                                                                // position
                                                                                .andThen(new InstantCommand(
                                                                                                () -> follower
                                                                                                                .holdPoint(follower
                                                                                                                                .getPose()))),

                                                                // Wait for turret to settle
                                                                new WaitCommand(100),

                                                                // Shoot (turret should be aligned)
                                                                new InstantCommand(() -> actuator.justShoot()),

                                                                // Release hold point and prepare for path 2
                                                                new InstantCommand(() -> follower.breakFollowing()),
                                                                new WaitCommand(200),

                                                                // Path 2: First sample pickup
                                                                new FollowPath(follower, path2).setMaxPower(1.0)
                                                                                .alongWith(
                                                                                                new SequentialCommandGroup(
                                                                                                                // At 1%
                                                                                                                // stop
                                                                                                                // shooter
                                                                                                                new WaitUntilPathPercent(
                                                                                                                                follower,
                                                                                                                                0.01)
                                                                                                                                .andThen(new InstantCommand(
                                                                                                                                                () -> actuator.stopShooter())),
                                                                                                                // At
                                                                                                                // 40%
                                                                                                                // run
                                                                                                                // intake
                                                                                                                new WaitUntilPathPercent(
                                                                                                                                follower,
                                                                                                                                0.40)
                                                                                                                                .andThen(new InstantCommand(
                                                                                                                                                () -> actuator.runIntake())),
                                                                                                                // Set
                                                                                                                // sorter
                                                                                                                new WaitCommand(400)
                                                                                                                                .andThen(new InstantCommand(
                                                                                                                                                () -> actuator.setSorterIN1())))),

                                                                // Path 3: Continue to sample with automatic ball
                                                                // detection
                                                                new FollowPath(follower, path3).setMaxPower(0.3)
                                                                                .alongWith(
                                                                                                new SequentialCommandGroup(
                                                                                                                // Start
                                                                                                                // automatic
                                                                                                                // ball
                                                                                                                // detection
                                                                                                                new InstantCommand(
                                                                                                                                () -> actuator.initBallDetection()),
                                                                                                                // Start
                                                                                                                // shooter
                                                                                                                new WaitCommand(10)
                                                                                                                                .andThen(new InstantCommand(
                                                                                                                                                () -> actuator.runShooterClose())),
                                                                                                                // Run
                                                                                                                // automatic
                                                                                                                // ball
                                                                                                                // detection
                                                                                                                new AutoBallDetection(
                                                                                                                                actuator))),
                                                                new WaitCommand(100),

                                                                // Path 4: Return to shooting position
                                                                new FollowPath(follower, path4).setMaxPower(1),

                                                                new WaitCommand(300),

                                                                // Path 5: Shoot first sample
                                                                new FollowPath(follower, path5).setMaxPower(1.0)
                                                                                .alongWith(
                                                                                                new SequentialCommandGroup(
                                                                                                                // Prepare
                                                                                                                // for
                                                                                                                // shooting
                                                                                                                new InstantCommand(
                                                                                                                                () -> actuator.FarGOready()),
                                                                                                                // Stop
                                                                                                                // intake
                                                                                                                // at
                                                                                                                // 90%
                                                                                                                new WaitUntilPathPercent(
                                                                                                                                follower,
                                                                                                                                0.90)
                                                                                                                                .andThen(new InstantCommand(
                                                                                                                                                () -> actuator.stopIntake()))))
                                                                                // Hold position
                                                                                .andThen(new InstantCommand(
                                                                                                () -> follower
                                                                                                                .holdPoint(follower
                                                                                                                                .getPose()))),

                                                                // Wait for turret alignment
                                                                new WaitCommand(100),

                                                                // Shoot
                                                                new InstantCommand(() -> actuator.FarGO()),

                                                                // Release hold point
                                                                new InstantCommand(() -> follower.breakFollowing()),
                                                                new WaitCommand(100),

                                                                // Path 6: Second sample pickup
                                                                new FollowPath(follower, path6).setMaxPower(1.0)
                                                                                .alongWith(
                                                                                                new SequentialCommandGroup(
                                                                                                                // Stop
                                                                                                                // shooter
                                                                                                                new WaitUntilPathPercent(
                                                                                                                                follower,
                                                                                                                                0.01)
                                                                                                                                .andThen(new InstantCommand(
                                                                                                                                                () -> actuator.stopShooter())),
                                                                                                                // Run
                                                                                                                // intake
                                                                                                                new WaitUntilPathPercent(
                                                                                                                                follower,
                                                                                                                                0.40)
                                                                                                                                .andThen(new InstantCommand(
                                                                                                                                                () -> actuator.runIntake())),
                                                                                                                // Set
                                                                                                                // sorter
                                                                                                                new WaitUntilPathPercent(
                                                                                                                                follower,
                                                                                                                                0.40)
                                                                                                                                .andThen(new InstantCommand(
                                                                                                                                                () -> actuator.setSorterIN1())))),

                                                                // Path 7: Continue to second sample
                                                                new FollowPath(follower, path7).setMaxPower(0.3)
                                                                                .alongWith(
                                                                                                new SequentialCommandGroup(
                                                                                                                // Start
                                                                                                                // ball
                                                                                                                // detection
                                                                                                                new InstantCommand(
                                                                                                                                () -> actuator.initBallDetection()),
                                                                                                                // Start
                                                                                                                // shooter
                                                                                                                new WaitCommand(10)
                                                                                                                                .andThen(new InstantCommand(
                                                                                                                                                () -> actuator.runShooterClose())),
                                                                                                                // Auto
                                                                                                                // ball
                                                                                                                // detection
                                                                                                                new AutoBallDetection(
                                                                                                                                actuator))),

                                                                // Path 8: Shoot second sample
                                                                new FollowPath(follower, path8).setMaxPower(1.0)
                                                                                .alongWith(
                                                                                                new SequentialCommandGroup(
                                                                                                                // Stop
                                                                                                                // intake
                                                                                                                new WaitCommand(600)
                                                                                                                                .andThen(new InstantCommand(
                                                                                                                                                () -> actuator.stopIntake())),
                                                                                                                // Prepare
                                                                                                                // for
                                                                                                                // mid
                                                                                                                // shot
                                                                                                                new WaitCommand(100)
                                                                                                                                .andThen(new InstantCommand(
                                                                                                                                                () -> actuator.MidGoready())),
                                                                                                                new WaitCommand(50)
                                                                                                                                .andThen(new InstantCommand(
                                                                                                                                                () -> actuator.setIntakePower(
                                                                                                                                                                1))),
                                                                                                                new WaitCommand(300)
                                                                                                                                .andThen(new InstantCommand(
                                                                                                                                                () -> actuator.stopIntake()))))
                                                                                // Hold position
                                                                                .andThen(new InstantCommand(
                                                                                                () -> follower
                                                                                                                .holdPoint(follower
                                                                                                                                .getPose()))),

                                                                // Wait for alignment
                                                                new WaitCommand(50),

                                                                // Shoot
                                                                new InstantCommand(() -> actuator.MidGO()),

                                                                new InstantCommand(() -> follower.breakFollowing()),
                                                                new WaitCommand(300),

                                                                // Path 9: Third sample pickup
                                                                new FollowPath(follower, path9).setMaxPower(1.0)
                                                                                .alongWith(
                                                                                                new SequentialCommandGroup(
                                                                                                                // Stop
                                                                                                                // shooter
                                                                                                                new WaitUntilPathPercent(
                                                                                                                                follower,
                                                                                                                                0.01)
                                                                                                                                .andThen(new InstantCommand(
                                                                                                                                                () -> actuator.stopShooter())),
                                                                                                                // Run
                                                                                                                // intake
                                                                                                                new WaitUntilPathPercent(
                                                                                                                                follower,
                                                                                                                                0.40)
                                                                                                                                .andThen(new InstantCommand(
                                                                                                                                                () -> actuator.runIntake())),
                                                                                                                // Set
                                                                                                                // sorter
                                                                                                                new WaitUntilPathPercent(
                                                                                                                                follower,
                                                                                                                                0.40)
                                                                                                                                .andThen(new InstantCommand(
                                                                                                                                                () -> actuator.setSorterIN1())))),

                                                                // Path 10: Continue to third sample
                                                                new FollowPath(follower, path10).setMaxPower(0.3)
                                                                                .alongWith(
                                                                                                new SequentialCommandGroup(
                                                                                                                // Start
                                                                                                                // ball
                                                                                                                // detection
                                                                                                                new InstantCommand(
                                                                                                                                () -> actuator.initBallDetection()),
                                                                                                                // Start
                                                                                                                // shooter
                                                                                                                new WaitCommand(10)
                                                                                                                                .andThen(new InstantCommand(
                                                                                                                                                () -> actuator.runShooterClose())),
                                                                                                                // Auto
                                                                                                                // ball
                                                                                                                // detection
                                                                                                                new AutoBallDetection(
                                                                                                                                actuator))),

                                                                // Path 11: Final shot
                                                                new FollowPath(follower, path11).setMaxPower(1.0)
                                                                                .alongWith(
                                                                                                new SequentialCommandGroup(
                                                                                                                // Stop
                                                                                                                // intake
                                                                                                                new WaitCommand(600)
                                                                                                                                .andThen(new InstantCommand(
                                                                                                                                                () -> actuator.stopIntake())),
                                                                                                                // Prepare
                                                                                                                // for
                                                                                                                // near
                                                                                                                // shot
                                                                                                                new WaitCommand(100)
                                                                                                                                .andThen(new InstantCommand(
                                                                                                                                                () -> actuator.NearGOready())),
                                                                                                                new WaitCommand(50)
                                                                                                                                .andThen(new InstantCommand(
                                                                                                                                                () -> actuator.setIntakePower(
                                                                                                                                                                1))),
                                                                                                                new WaitCommand(300)
                                                                                                                                .andThen(new InstantCommand(
                                                                                                                                                () -> actuator.stopIntake()))))
                                                                                // Hold position
                                                                                .andThen(new InstantCommand(
                                                                                                () -> follower
                                                                                                                .holdPoint(follower
                                                                                                                                .getPose()))),

                                                                // Wait for alignment
                                                                new WaitCommand(100),

                                                                // Shoot
                                                                new InstantCommand(() -> actuator.NearGO()),

                                                                new InstantCommand(() -> follower.breakFollowing()),
                                                                new WaitCommand(300),

                                                                // Path 12: Park
                                                                new FollowPath(follower, path12).setMaxPower(1.0)
                                                                                .andThen(new InstantCommand(
                                                                                                () -> follower
                                                                                                                .holdPoint(follower
                                                                                                                                .getPose())))),

                                                // CONTINUOUS TURRET TRACKING - runs in parallel with ALL commands above
                                                new RunCommand(() -> {
                                                        Pose pose = follower.getPose();
                                                        if (pose != null) {
                                                                turret.update(pose);
                                                        }
                                                })));
        }

        @Override
        public void init_loop() {
                super.init_loop();
                // Scan for AprilTags during init
                tagSubsystem.update();
        }

        @Override
        public void iter() {
                // This is called every loop - telemetry is handled by the RunCommand
        }

        @Override
        public void end() {
                // Save final pose for TeleOp
                Pose finalPose = follower.getPose();
                if (finalPose != null) {
                        GlobalData.autonomousEndPose = finalPose;
                        GlobalData.hasAutonomousRun = true;
                }

                // Save turret tracking state for TeleOp
                if (turret != null) {
                        GlobalData.turretTrackingEnabled = true; // Keep tracking enabled
                        GlobalData.turretAngleOffset = turret.getAngleOffset();
                }

                // Stop all subsystems
                if (tagSubsystem != null) {
                        tagSubsystem.stop();
                }
                if (actuator != null) {
                        actuator.stopShooter();
                        actuator.stopIntake();
                }
                if (turret != null) {
                        turret.stop();
                }
        }

        // Path builders (same as RedFar12Art)
        private PathChain buildPath1() {
                return new PathBuilder(follower, Constants.pathConstraints)
                                .addPath(new BezierLine(
                                                new Pose(110.5, 129.5),
                                                new Pose(93.35, 94.5)))
                                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(45))
                                .build();
        }

        private PathChain buildPath2() {
                return new PathBuilder(follower, Constants.pathConstraints)
                                .addPath(new BezierCurve(
                                                new Pose(93.35, 94.5),
                                                new Pose(90.923, 84.321),
                                                new Pose(95.512, 83.335)))
                                .setLinearHeadingInterpolation(Math.toRadians(50), Math.toRadians(2))
                                .build();
        }

        private PathChain buildPath3() {
                return new PathBuilder(follower, Constants.pathConstraints)
                                .addPath(new BezierLine(
                                                new Pose(95.512, 83.335),
                                                new Pose(117.971, 83.246)))
                                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(2))
                                .build();
        }

        private PathChain buildPath4() {
                return new PathBuilder(follower, Constants.pathConstraints)
                                .addPath(new BezierCurve(
                                                new Pose(117.971, 85.246),
                                                new Pose(121.17812061711079, 83.99719495091163),
                                                new Pose(117.94670406732118, 76.12061711079943),
                                                new Pose(126.602, 73.00)))
                                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(95))
                                .build();
        }

        private PathChain buildPath5() {
                return new PathBuilder(follower, Constants.pathConstraints)
                                .addPath(new BezierLine(
                                                new Pose(126.602, 73.00),
                                                new Pose(93.35, 86)))
                                .setLinearHeadingInterpolation(Math.toRadians(95), Math.toRadians(41))
                                .build();
        }

        private PathChain buildPath6() {
                return new PathBuilder(follower, Constants.pathConstraints)
                                .addPath(new BezierCurve(
                                                new Pose(93.35, 86),
                                                new Pose(86.174, 62.948),
                                                new Pose(93.332, 60.320)))
                                .setLinearHeadingInterpolation(Math.toRadians(41), Math.toRadians(0))
                                .build();
        }

        private PathChain buildPath7() {
                return new PathBuilder(follower, Constants.pathConstraints)
                                .addPath(new BezierLine(
                                                new Pose(93.332, 60.320),
                                                new Pose(123, 60.320)))
                                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                                .build();
        }

        private PathChain buildPath8() {
                return new PathBuilder(follower, Constants.pathConstraints)
                                .addPath(new BezierLine(
                                                new Pose(123, 59.320),
                                                new Pose(93.5, 86)))
                                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(38))
                                .build();
        }

        private PathChain buildPath9() {
                return new PathBuilder(follower, Constants.pathConstraints)
                                .addPath(new BezierCurve(
                                                new Pose(93.5, 86),
                                                new Pose(91.71107994389902, 46.451612903225815),
                                                new Pose(98.795, 38.216)))
                                .setLinearHeadingInterpolation(Math.toRadians(38), Math.toRadians(0))
                                .build();
        }

        private PathChain buildPath10() {
                return new PathBuilder(follower, Constants.pathConstraints)
                                .addPath(new BezierLine(
                                                new Pose(98.795, 38.216),
                                                new Pose(120.254, 38.306)))
                                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                                .build();
        }

        private PathChain buildPath11() {
                return new PathBuilder(follower, Constants.pathConstraints)
                                .addPath(new BezierLine(
                                                new Pose(120.254, 34.306),
                                                new Pose(92.35, 88)))
                                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(38))
                                .build();
        }

        private PathChain buildPath12() {
                return new PathBuilder(follower, Constants.pathConstraints)
                                .addPath(new BezierLine(
                                                new Pose(93.35, 86),
                                                new Pose(100, 76)))
                                .setLinearHeadingInterpolation(Math.toRadians(38), Math.toRadians(41))
                                .build();
        }
}
