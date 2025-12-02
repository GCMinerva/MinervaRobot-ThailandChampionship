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

@Autonomous(name = "RedFar12Art", group = "Auton")
public class RedFar12Art extends OpModeCommand {

        private Follower follower;
        private AprilTagSubsystem tagSubsystem;
        private ActuatorSubsystem actuator;
        private TurretSubsystem turret;

        // Starting pose
        private final Pose startPose = new Pose(110.5, 129.5, Math.toRadians(90));

        @Override
        public void initialize() {
                // === Initialize Subsystems ===
                tagSubsystem = new AprilTagSubsystem(hardwareMap, telemetry);
                actuator = new ActuatorSubsystem(hardwareMap, telemetry);
                turret = new TurretSubsystem(hardwareMap);
                follower = Constants.createFollower(hardwareMap);

                // telemetry.addLine("🔍 Scanning for AprilTags...");
                // telemetry.addLine("⚙️ Actuator ready");
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

                // Schedule command sequence
                schedule(
                                // Telemetry display - COMMENTED OUT FOR TESTING
                                /*
                                 * new RunCommand(() -> {
                                 * Pose p = follower.getPose();
                                 * telemetry.addData("Pose", "X: %.1f, Y: %.1f, H: %.1f°",
                                 * (double)p.getX(), (double)p.getY(), (double)Math.toDegrees(p.getHeading()));
                                 * telemetry.addData("Follower Busy", follower.isBusy());
                                 * telemetry.addData("Path Progress", "%.1f%%",
                                 * (double)(follower.getCurrentTValue() * 100));
                                 * telemetry.update();
                                 * }),
                                 */

                                // Main autonomous sequence
                                new SequentialCommandGroup(
                                                // Wait for start
                                                new WaitCommand(10),

                                                // Path 1: Initial scoring - complete path THEN shoot while stationary
                                                new FollowPath(follower, path1)
                                                                .setMaxPower(1)
                                                                .alongWith(
                                                                                new SequentialCommandGroup(
                                                                                                new WaitCommand(20)
                                                                                                                .andThen(new InstantCommand(
                                                                                                                                () -> actuator.runOuttake(-0.7))),

                                                                                                new WaitCommand(60) // 1400
                                                                                                                .andThen(new InstantCommand(
                                                                                                                                () -> turret.turnToAngle(
                                                                                                                                                90))),
                                                                                                new WaitCommand(200)
                                                                                                                .andThen(new InstantCommand(
                                                                                                                                () -> actuator.runShooterClose()))
                                                                                                // At 1% of path, start
                                                                                                // shooter to spin up
                                                                                                ))
                                                                // After path completes, hold current position
                                                                .andThen(new InstantCommand(() -> follower
                                                                                .holdPoint(follower.getPose())))
                                                                // Then shoot while stationary
                                                                .andThen(new InstantCommand(
                                                                                () -> actuator.justShoot())),
                                                // .andThen(new WaitCommand(4000)), // Wait 4 seconds for justShoot() to
                                                // complete

                                                // Release hold point and prepare for path 2
                                                new InstantCommand(() -> follower.breakFollowing()),
                                                new WaitCommand(500), // Short wait to ensure follower is ready

                                                // Path 2: First sample pickup
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

                                                // Path 3: Continue to sample with automatic ball detection
                                                new FollowPath(follower, path3).setMaxPower(0.28) // 21
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

                                                // Path 4: Return to shooting position
                                                new FollowPath(follower, path4).setMaxPower(0.9),

                                                new WaitCommand(300),

                                                // Path 5: Shoot first sample
                                                new FollowPath(follower, path5).setMaxPower(1.0)
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
                                                                                () -> actuator.FarGO())),
                                                // .andThen(new WaitCommand(4000)), // Wait 4 seconds for justShoot() to
                                                // complete

                                                // Release hold point and prepare for path 2
                                                new InstantCommand(() -> follower.breakFollowing()),
                                                new WaitCommand(400),
                                                // // .andThen(new WaitCommand(4000)), // Wait 4 seconds for justShoot()
                                                // to complete
                                                //
                                                // ========== PATHS 6-11 COMMENTED OUT FOR TESTING ==========
                                                // Uncomment when ready to test full autonomous sequence

                                                // /*

                                                // Path 6: Second sample pickup
                                                new FollowPath(follower, path6).setMaxPower(1.0)
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

                                                // Path 7: Continue to second sample
                                                new FollowPath(follower, path7).setMaxPower(0.27)
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

                                                // /*
                                                // Path 8: Shoot second sample
                                                new FollowPath(follower, path8).setMaxPower(1.0)
                                                                .alongWith(
                                                                                new SequentialCommandGroup(
                                                                                                // At 10% stop intake
                                                                                                new InstantCommand(
                                                                                                                () -> actuator.MidGoready()),

                                                                                                new WaitCommand(500)
                                                                                                                .andThen(new InstantCommand(
                                                                                                                                () -> actuator.stopIntake()))

                                                                                ))
                                                                // // After path completes, hold current position
                                                                .andThen(new InstantCommand(() -> follower
                                                                                .holdPoint(follower.getPose())))
                                                // // Then shoot while stationary
                                                ,
                                                // // Then shoot while stationary
                                                new WaitCommand(100)
                                                                .andThen(new InstantCommand(() -> actuator.MidGO())),

                                                new InstantCommand(() -> follower.breakFollowing()),
                                                new WaitCommand(300),

                                                new FollowPath(follower, path9).setMaxPower(1.0)
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



                                        new FollowPath(follower, path10).setMaxPower(0.29)
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

            new FollowPath(follower, path11).setMaxPower(1.0)
                    .alongWith(
                            new SequentialCommandGroup(
                                    // At 10% stop intake
                                    new InstantCommand(
                                            () -> actuator.NearGOready()),

                                    new WaitCommand(500)
                                            .andThen(new InstantCommand(
                                                    () -> actuator.stopIntake()))

                            ))
                    // // After path completes, hold current position
                    .andThen(new InstantCommand(() -> follower
                            .holdPoint(follower.getPose()))),

                                        // // Then shoot while stationary
                                        new WaitCommand(100)
                                                .andThen(new InstantCommand(() -> actuator.NearGO())),

                                        new InstantCommand(() -> follower.breakFollowing()),
                                        new WaitCommand(300),

                                        new FollowPath(follower, path12).setMaxPower(1.0)

                                        .andThen(new InstantCommand(() -> follower
                                                .holdPoint(follower.getPose())))



                                /*

                                 * // Path 10: Continue to third sample
                                 * new FollowPath(follower, path10).setMaxPower(0.25)
                                 * .alongWith(
                                 * new SequentialCommandGroup(
                                 * // At 35% sorter IN2
                                 * new WaitUntilPathPercent(follower, 0.35)
                                 * .andThen(new InstantCommand(() -> actuator.setSorterIN2())),
                                 * // At 60% sorter IN3
                                 * new WaitUntilPathPercent(follower, 0.60)
                                 * .andThen(new InstantCommand(() -> actuator.setSorterIN3())),
                                 * // At 95% stop intake
                                 * new WaitUntilPathPercent(follower, 0.95)
                                 * .andThen(new InstantCommand(() -> actuator.stopIntake())),
                                 * // At 100% start shooter
                                 * new WaitUntilPathPercent(follower, 1.00)
                                 * .andThen(new InstantCommand(() -> actuator.runShooterClose()))
                                 * )
                                 * ),
                                 * 
                                 * // Path 11: Final shot
                                 * new FollowPath(follower, path11).setMaxPower(1.0)
                                 * .alongWith(
                                 * new SequentialCommandGroup(
                                 * // At 100% shoot
                                 * new WaitUntilPathPercent(follower, 1.00)
                                 * .andThen(new InstantCommand(() -> actuator.NearGO()))
                                 * )
                                 * )
                                 * .andThen(new WaitCommand(300))
                                 */

                                ));
        }

        @Override
        public void init_loop() {
                super.init_loop();
                // Scan for AprilTags during init
                tagSubsystem.update();
                // telemetry.addData("Status", "Scanning for AprilTags...");
                // telemetry.addData("Detected Tag", GlobalData.detectedTagId);
                // telemetry.update();
        }

        @Override
        public void iter() {
                // This is called every loop - telemetry is handled by the RunCommand
        }

        @Override
        public void end() {
                // Save final pose for Manual OpMode
                Pose finalPose = follower.getPose();
                if (finalPose != null) {
                        GlobalData.autonomousEndPose = finalPose;
                        GlobalData.hasAutonomousRun = true;
                }

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
                                                new Pose(93.35, 94.5)))
                                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(52))
                                .build();
        }

        private PathChain buildPath2() {
                return new PathBuilder(follower, Constants.pathConstraints)
                                .addPath(new BezierCurve(
                                                new Pose(93.35, 94.5),
                                                new Pose(92.923, 84.321),
                                                new Pose(100.512, 83.335)))
                                .setLinearHeadingInterpolation(Math.toRadians(48), Math.toRadians(2))
                                .build();
        }

        private PathChain buildPath3() {
                return new PathBuilder(follower, Constants.pathConstraints)
                                .addPath(new BezierLine(
                                                new Pose(100.512, 83.335),
                                                new Pose(117.971, 83.246)))
                                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(2))
                                .build();
        }

        private PathChain buildPath4() {
                return new PathBuilder(follower, Constants.pathConstraints)
                                .addPath(new BezierCurve(
                                                new Pose(117.971, 85.246),
                                                new Pose(110.65638148667603, 85.83450210378682),
                                                new Pose(107.85834502103786, 71.273492286115),
                                                new Pose(127.602, 71.00)))
                                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(95))
                                .build();
        }

        private PathChain buildPath5() {
                return new PathBuilder(follower, Constants.pathConstraints)
                                .addPath(new BezierLine(
                                                new Pose(127.602, 70.00),
                                                new Pose(93.35, 86)))
                                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(40))
                                .build();
        }

        private PathChain buildPath6() {
                return new PathBuilder(follower, Constants.pathConstraints)
                                .addPath(new BezierCurve(
                                                new Pose(93.35, 86),
                                                new Pose(88.174, 59.948),
                                                new Pose(100.332, 61.320)))
                                .setLinearHeadingInterpolation(Math.toRadians(42), Math.toRadians(0))
                                .build();
        }

        private PathChain buildPath7() {
                return new PathBuilder(follower, Constants.pathConstraints)
                                .addPath(new BezierLine(
                                                new Pose(100.332, 61.320),
                                                new Pose(123, 61.320)))
                                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                                .build();
        }

        private PathChain buildPath8() {
                return new PathBuilder(follower, Constants.pathConstraints)
                                .addPath(new BezierLine(
                                                new Pose(123, 61.320),
                                                new Pose(93.5, 86)))
                                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(44))
                                .build();
        }

        private PathChain buildPath9() {
                return new PathBuilder(follower, Constants.pathConstraints)
                                .addPath(new BezierCurve(
                                                new Pose(93.5, 86),
                                                new Pose(75.450, 44.714),
                                                new Pose(98.795, 36.216)))
                                .setLinearHeadingInterpolation(Math.toRadians(48), Math.toRadians(0))
                                .build();
        }

        private PathChain buildPath10() {
                return new PathBuilder(follower, Constants.pathConstraints)
                                .addPath(new BezierLine(
                                                new Pose(98.795, 36.216),
                                                new Pose(120.254, 36.306)))
                                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                                .build();
        }

        private PathChain buildPath11() {
                return new PathBuilder(follower, Constants.pathConstraints)
                                .addPath(new BezierLine(
                                                new Pose(120.254, 32.306),
                                                new Pose(93.35, 86)))
                                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(44))
                                .build();
        }
        private PathChain buildPath12() {
                return new PathBuilder(follower, Constants.pathConstraints)
                                .addPath(new BezierLine(
                                        new Pose(93.35, 86),
                                        new Pose(100, 76)))
                                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(44))
                                .build();
    }
}
