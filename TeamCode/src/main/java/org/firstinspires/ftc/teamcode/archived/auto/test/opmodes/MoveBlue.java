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

@Autonomous(name = "MoveBlue", group = "Auton")
public class MoveBlue extends OpModeCommand {

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
                        new FollowPath(follower, path1).setMaxPower(1)
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
                        new Pose(110.5, 129.5),
                        new Pose(75, 129.5)))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
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
