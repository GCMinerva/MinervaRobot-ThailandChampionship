package org.firstinspires.ftc.teamcode.archived.auto.test.opmodes.Blue;

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

@Autonomous(name = "BlueFar[12Art]", group = "Auton")
public class BlueFar12Art extends OpModeCommand {

        private Follower follower;
        private AprilTagSubsystem tagSubsystem;
        private ActuatorSubsystem actuator;
        private TurretSubsystem turret;

        // Starting pose
        private final Pose startPose = new Pose(107.5, 126.5, Math.toRadians(90)).mirror();

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
                                                                    () -> actuator.runOuttake(-0.81))),

                                                    new WaitCommand(60) // 1400
                                                            .andThen(new InstantCommand(
                                                                    () -> turret.turnToAngle(
                                                                            -90))),
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
                            new WaitCommand(200), // Short wait to ensure follower is ready

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
                                                    new WaitCommand(400)
                                                            .andThen(new InstantCommand(
                                                                    () -> actuator.setSorterIN1())))),

                            // Path 3: Continue to sample with automatic ball detection
                            new FollowPath(follower, path3).setMaxPower(0.26) // 21
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
                            new FollowPath(follower, path4).setMaxPower(1),

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
                            new WaitCommand(100),
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
                            new FollowPath(follower, path7).setMaxPower(0.23)
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

                                                    new WaitCommand(600)
                                                            .andThen(new InstantCommand(
                                                                    () -> actuator.stopIntake())),
                                                    // At 10% stop intake
                                                    new WaitCommand(100).andThen( new InstantCommand(() -> actuator.MidGoready())
                                                    ),
                                                    new WaitCommand(50).andThen(new InstantCommand(  () -> actuator.setIntakePower(1))),
                                                    new WaitCommand(300).andThen(new InstantCommand( () -> actuator.stopIntake()))



                                            ))
                                    // // After path completes, hold current position
                                    .andThen(new InstantCommand(() -> follower
                                            .holdPoint(follower.getPose())))
                            // // Then shoot while stationary
                            ,
                            // // Then shoot while stationary
                            new WaitCommand(50)
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



                            new FollowPath(follower, path10).setMaxPower(0.25)
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

                                                    new WaitCommand(700)
                                                            .andThen(new InstantCommand(
                                                                    () -> actuator.stopIntake())),
                                                    // At 10% stop intake
                                                    new WaitCommand(100).andThen( new InstantCommand(() -> actuator.NearGOready())
                                                    ),
                                                    new WaitCommand(50).andThen(new InstantCommand(  () -> actuator.setIntakePower(1))),
                                                    new WaitCommand(300).andThen(new InstantCommand( () -> actuator.stopIntake()))



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
                                 *
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
                                                new Pose(107.5, 126.5).mirror(),
                                                new Pose(93.35, 94.5).mirror()))
                                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(123))
                                .build();
        }

        private PathChain buildPath2() {
                return new PathBuilder(follower, Constants.pathConstraints)
                                .addPath(new BezierCurve(
                                                new Pose(93.35, 94.5).mirror(),
                                                new Pose(92.923, 84.321).mirror(),
                                                new Pose(100.512, 81.335).mirror()))
                                .setLinearHeadingInterpolation(Math.toRadians(123), Math.toRadians(180))
                                .build();
        }

        private PathChain buildPath3() {
                return new PathBuilder(follower, Constants.pathConstraints)
                                .addPath(new BezierLine(
                                                new Pose(100.512, 81.335).mirror(),
                                                new Pose(117.971, 81.246).mirror()))
                                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                                .build();
        }

        private PathChain buildPath4() {
                return new PathBuilder(follower, Constants.pathConstraints)
                                .addPath(new BezierCurve(
                                                new Pose(117.971, 81.246).mirror(),
                                                new Pose(121.17812061711079, 83.99719495091163).mirror(),
                                                new Pose(117.94670406732118, 76.12061711079943).mirror(),
                                                new Pose(123.602, 73.00).mirror()))
                                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(85))
                                .build();
        }

        private PathChain buildPath5() {
                return new PathBuilder(follower, Constants.pathConstraints)
                                .addPath(new BezierLine(
                                                new Pose(123.602, 73.00).mirror(),
                                                new Pose(93.35, 86).mirror()))
                                .setLinearHeadingInterpolation(Math.toRadians(85), Math.toRadians(133))
                                .build();
        }

        private PathChain buildPath6() {
                return new PathBuilder(follower, Constants.pathConstraints)
                                .addPath(new BezierCurve(
                                                new Pose(93.35, 86).mirror(),
                                                new Pose(88.174, 59.948).mirror(),
                                                new Pose(98.332, 57.320).mirror()))
                                .setLinearHeadingInterpolation(Math.toRadians(133), Math.toRadians(180))
                                .build();
        }

        private PathChain buildPath7() {
                return new PathBuilder(follower, Constants.pathConstraints)
                                .addPath(new BezierLine(
                                                new Pose(98.332, 57.320).mirror(),
                                                new Pose(121, 57.320).mirror()))
                                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                                .build();
        }

        private PathChain buildPath8() {
                return new PathBuilder(follower, Constants.pathConstraints)
                                .addPath(new BezierLine(
                                                new Pose(121, 57.320).mirror(),
                                                new Pose(93.5, 83).mirror()))
                                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(145))
                                .build();
        }

        private PathChain buildPath9() {
                return new PathBuilder(follower, Constants.pathConstraints)
                                .addPath(new BezierCurve(
                                                new Pose(93.5, 83).mirror(),
                                                new Pose(93.71107994389902, 46.451612903225815).mirror(),
                                                new Pose(103.795, 34.216).mirror()))
                                .setLinearHeadingInterpolation(Math.toRadians(145), Math.toRadians(180))
                                .build();
        }

        private PathChain buildPath10() {
                return new PathBuilder(follower, Constants.pathConstraints)
                                .addPath(new BezierLine(
                                                new Pose(103.795, 34.216).mirror(),
                                                new Pose(120.254, 34.306).mirror()))
                                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                                .build();
        }

        private PathChain buildPath11() {
                return new PathBuilder(follower, Constants.pathConstraints)
                                .addPath(new BezierLine(
                                                new Pose(120.254, 34.306).mirror(),
                                                new Pose(93.35, 83.5).mirror()))
                                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(146))
                                .build();
        }

        private PathChain buildPath12() {
                return new PathBuilder(follower, Constants.pathConstraints)
                                .addPath(new BezierLine(
                                                new Pose(93.35, 83.5).mirror(),
                                                new Pose(104, 77).mirror()))
                                .setLinearHeadingInterpolation(Math.toRadians(146), Math.toRadians(143))
                                .build();
        }
}
