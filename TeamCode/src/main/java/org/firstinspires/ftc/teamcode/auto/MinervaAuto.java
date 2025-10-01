package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.lang.reflect.Method;

@Autonomous(name = "Minerva Auto", group = "Auto")
public class MinervaAuto extends OpMode {

    // ============================================
    // CHANGE THIS TO SELECT WHICH PATH TO RUN
    // Just type the filename (e.g., "Path1", "Path2", "TestPath")
    // ============================================
    private static final String SELECTED_PATH = "Path1";

    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState;

    private final Pose startPose = new Pose(56.000, 8.000, Math.toRadians(90));

    private PathChain selectedPath;

    public void buildPaths() {
        try {
            // Dynamically load the path class by name
            String className = "org.firstinspires.ftc.teamcode.auto.paths." + SELECTED_PATH;
            Class<?> pathClass = Class.forName(className);

            // Get the buildPath method
            Method buildMethod = pathClass.getMethod("buildPath", Follower.class, com.pedropathing.paths.PathConstraints.class);

            // Call the buildPath method
            selectedPath = (PathChain) buildMethod.invoke(null, follower, Constants.pathConstraints);

            telemetry.addData("Path Loaded", SELECTED_PATH);
            telemetry.update();
        } catch (Exception e) {
            telemetry.addData("ERROR", "Path '" + SELECTED_PATH + "' not found!");
            telemetry.addData("Error Details", e.getMessage());
            telemetry.update();
        }
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                if (selectedPath != null) {
                    follower.followPath(selectedPath);
                    setPathState(1);
                }
                break;
            case 1:
                if (!follower.isBusy()) {
                    setPathState(-1);
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        telemetry.addData("Selected Path", SELECTED_PATH);
        telemetry.addData("Path State", pathState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Elapsed Time", opmodeTimer.getElapsedTimeSeconds());
        telemetry.update();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void stop() {}
}