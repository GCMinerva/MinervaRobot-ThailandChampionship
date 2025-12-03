package org.firstinspires.ftc.teamcode.archived.auto.test.opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.archived.auto.test.commands.FollowPath;
import org.firstinspires.ftc.teamcode.archived.auto.test.commands.OpModeCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

@Autonomous(name = "RedFar[KAO] - Only Drive", group = "Auton")
public class RED2kao extends OpModeCommand {

    private Follower follower;

    // Starting pose
    private final Pose startPose = new Pose(87, 9.5, Math.toRadians(90));

    @Override
    public void initialize() {
        follower = Constants.createFollower(hardwareMap);

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
                        new FollowPath(follower, path1).setMaxPower(1.0),
                        new FollowPath(follower, path2).setMaxPower(1.0),
                        new FollowPath(follower, path3).setMaxPower(0.26),
                        new FollowPath(follower, path4).setMaxPower(1.0),
                        new FollowPath(follower, path5).setMaxPower(1.0),
                        new FollowPath(follower, path6).setMaxPower(0.26),
                        new FollowPath(follower, path7).setMaxPower(1.0),
                        new FollowPath(follower, path8).setMaxPower(1.0),
                        new FollowPath(follower, path9).setMaxPower(0.26),
                        new FollowPath(follower, path10).setMaxPower(1.0)
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
    private PathChain buildPath1() {
        return new PathBuilder(follower, Constants.pathConstraints)
                .addPath(new BezierLine(new Pose(87.000, 9.500), new Pose(87.000, 19.500)))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(67))
                .build();
    }
    private PathChain buildPath2() {
        return new PathBuilder(follower, Constants.pathConstraints)
                .addPath(new BezierCurve(
                        new Pose(87.000, 19.500),
                        new Pose(86.541, 34.710),
                        new Pose(99.909, 35.414)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(67), Math.toRadians(0))
                .build();
    }

    private PathChain buildPath3() {
        return new PathBuilder(follower, Constants.pathConstraints)
                .addPath(new BezierLine(new Pose(99.909, 35.414), new Pose(124.769, 35.414)))
                .setTangentHeadingInterpolation()
                .build();
    }

    private PathChain buildPath4() {
        return new PathBuilder(follower, Constants.pathConstraints)
                .addPath(new BezierLine(new Pose(124.769, 35.414), new Pose(87.010, 9.381)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(67))
                .build();
    }

    private PathChain buildPath5() {
        return new PathBuilder(follower, Constants.pathConstraints)
                .addPath(new BezierCurve(
                        new Pose(87.010, 9.381),
                        new Pose(84.899, 60.274),
                        new Pose(99.440, 59.336)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(67), Math.toRadians(0))
                .build();
    }

    private PathChain buildPath6() {
        return new PathBuilder(follower, Constants.pathConstraints)
                .addPath(new BezierLine(new Pose(99.440, 59.336), new Pose(125.003, 59.336)))
                .setTangentHeadingInterpolation()
                .build();
    }

    private PathChain buildPath7() {
        return new PathBuilder(follower, Constants.pathConstraints)
                .addPath(new BezierCurve(
                        new Pose(125.003, 59.336),
                        new Pose(85.603, 68.717),
                        new Pose(87.948, 87.479)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                .build();
    }

    private PathChain buildPath8() {
        return new PathBuilder(follower, Constants.pathConstraints)
                .addPath(new BezierCurve(
                        new Pose(87.948, 87.479),
                        new Pose(88.417, 82.554),
                        new Pose(99.909, 83.257)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                .build();
    }

    private PathChain buildPath9() {
        return new PathBuilder(follower, Constants.pathConstraints)
                .addPath(new BezierLine(new Pose(99.909, 83.257), new Pose(124.769, 83.492)))
                .setTangentHeadingInterpolation()
                .build();
    }

    private PathChain buildPath10() {
        return new PathBuilder(follower, Constants.pathConstraints)
                .addPath(new BezierLine(new Pose(124.769, 83.492), new Pose(87.479, 87.244)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                .build();
    }
}
