package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous(name = "Minerva Auto Draff 1", group = "Auto")
public class MinervaAutoDraff1 extends OpMode {

   private Follower follower;
   private Timer pathTimer, opmodeTimer, waitTimer;
   private int pathState;
   private RobotConfig robotConfig;

   // AprilTag vision
   private AprilTagProcessor aprilTag;
   private VisionPortal visionPortal;
   private int detectedAprilTagId = -1;

   // Configuration constants - change these to customize behavior
   private static final double START_X = 56.000;
   private static final double START_Y = 8.000;
   private static final double START_HEADING = 90; // degrees

   // Shooter configuration
   private static final double SHOOTER_TARGET_X = 11.0;  // Shooting target X position
   private static final double SHOOTER_TARGET_Y = 135.0; // Shooting target Y position
   private static final double SHOOTER_SPIN_UP_TIME = 0.5;  // Time for shooter to spin up
   private static final double SHOOTER_FEED_TIME = 0.3;     // Time for ball to feed
   private static final double SHOOTER_RESET_TIME = 0.3;    // Time for servo to reset

   private final Pose startPose = new Pose(START_X, START_Y, Math.toRadians(START_HEADING));

   // Individual path chains
   private PathChain path1, path2, path3, path4, path5, path6, path7, path8;

   public void buildPaths() {
       PathBuilder builder = new PathBuilder(follower, Constants.pathConstraints);

       // Path 1 - followed by wait
       path1 = builder
           .addPath(new BezierLine(new Pose(56.000, 8.000), new Pose(56.366, 19.356)))
           .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(115))
           .build();

       // Path 2
       path2 = builder
           .addPath(
               new BezierCurve(
                   new Pose(56.366, 19.356),
                   new Pose(48.284, 37.861),
                   new Pose(17.867, 36.585)
               )
           )
           .setLinearHeadingInterpolation(Math.toRadians(115), Math.toRadians(180))
           .build();

       // Path 3 - followed by wait
       path3 = builder
           .addPath(new BezierLine(new Pose(17.867, 36.585), new Pose(68.490, 26.162)))
           .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(120))
           .build();

       // Path 4
       path4 = builder
           .addPath(
               new BezierCurve(
                   new Pose(68.490, 26.162),
                   new Pose(24.674, 44.455),
                   new Pose(25.099, 57.855)
               )
           )
           .setLinearHeadingInterpolation(Math.toRadians(120), Math.toRadians(90))
           .build();

       // Path 5 - followed by wait
       path5 = builder
           .addPath(
               new BezierCurve(
                   new Pose(25.099, 57.855),
                   new Pose(34.245, 78.275),
                   new Pose(70.617, 75.935)
               )
           )
           .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(135))
           .build();

       // Path 6
       path6 = builder
           .addPath(new BezierLine(new Pose(70.617, 75.935), new Pose(15.527, 69.767)))
           .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(90))
           .build();

       // Path 7
       path7 = builder
           .addPath(
               new BezierCurve(
                   new Pose(15.527, 69.767),
                   new Pose(20.419, 91.462),
                   new Pose(35.521, 91.250)
               )
           )
           .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(15))
           .build();

       // Path 8 - followed by wait
       path8 = builder
           .addPath(
               new BezierCurve(
                   new Pose(35.521, 91.250),
                   new Pose(52.538, 89.973),
                   new Pose(50.411, 99.545)
               )
           )
           .setLinearHeadingInterpolation(Math.toRadians(15), Math.toRadians(135))
           .build();
   }

   public void autonomousPathUpdate() {
       switch (pathState) {
           case 0:
               // Execute Path 1
               follower.followPath(path1);
               setPathState(1);
               break;
           case 1:
               // Wait for Path 1 to complete
               if (!follower.isBusy()) {
                   follower.breakFollowing();
                   waitTimer.resetTimer();
                   setPathState(2);
               }
               break;
           case 2:
               // Shooting Position 1 - Prepare shooter
               robotConfig.shooter.prepareShot(follower.getPose());
               setPathState(21);
               break;
           case 21:
               // Shooting Position 1 - Wait for spin up
               if (waitTimer.getElapsedTimeSeconds() >= SHOOTER_SPIN_UP_TIME) {
                   robotConfig.shooter.shoot();
                   setPathState(22);
               }
               break;
           case 22:
               // Shooting Position 1 - Wait for ball to shoot
               if (waitTimer.getElapsedTimeSeconds() >= SHOOTER_FEED_TIME) {
                   robotConfig.shooter.reset();
                   setPathState(23);
               }
               break;
           case 23:
               // Shooting Position 1 - Wait for servo reset, then continue
               if (waitTimer.getElapsedTimeSeconds() >= SHOOTER_RESET_TIME) {
                   robotConfig.shooter.stopAll();
                   follower.followPath(path2);
                   setPathState(3);
               }
               break;
           case 3:
               // Wait for Path 2 to complete
               if (!follower.isBusy()) {
                   follower.followPath(path3);
                   setPathState(4);
               }
               break;
           case 4:
               // Wait for Path 3 to complete
               if (!follower.isBusy()) {
                   follower.breakFollowing();
                   waitTimer.resetTimer();
                   setPathState(5);
               }
               break;
           case 5:
               // Shooting Position 2 - Prepare shooter
               robotConfig.shooter.prepareShot(follower.getPose());
               setPathState(51);
               break;
           case 51:
               // Shooting Position 2 - Wait for spin up
               if (waitTimer.getElapsedTimeSeconds() >= SHOOTER_SPIN_UP_TIME) {
                   robotConfig.shooter.shoot();
                   setPathState(52);
               }
               break;
           case 52:
               // Shooting Position 2 - Wait for ball to shoot
               if (waitTimer.getElapsedTimeSeconds() >= SHOOTER_FEED_TIME) {
                   robotConfig.shooter.reset();
                   setPathState(53);
               }
               break;
           case 53:
               // Shooting Position 2 - Wait for servo reset, then continue
               if (waitTimer.getElapsedTimeSeconds() >= SHOOTER_RESET_TIME) {
                   robotConfig.shooter.stopAll();
                   follower.followPath(path4);
                   setPathState(6);
               }
               break;
           case 6:
               // Wait for Path 4 to complete
               if (!follower.isBusy()) {
                   follower.followPath(path5);
                   setPathState(7);
               }
               break;
           case 7:
               // Wait for Path 5 to complete
               if (!follower.isBusy()) {
                   follower.breakFollowing();
                   waitTimer.resetTimer();
                   setPathState(8);
               }
               break;
           case 8:
               // Shooting Position 3 - Prepare shooter
               robotConfig.shooter.prepareShot(follower.getPose());
               setPathState(81);
               break;
           case 81:
               // Shooting Position 3 - Wait for spin up
               if (waitTimer.getElapsedTimeSeconds() >= SHOOTER_SPIN_UP_TIME) {
                   robotConfig.shooter.shoot();
                   setPathState(82);
               }
               break;
           case 82:
               // Shooting Position 3 - Wait for ball to shoot
               if (waitTimer.getElapsedTimeSeconds() >= SHOOTER_FEED_TIME) {
                   robotConfig.shooter.reset();
                   setPathState(83);
               }
               break;
           case 83:
               // Shooting Position 3 - Wait for servo reset, then continue
               if (waitTimer.getElapsedTimeSeconds() >= SHOOTER_RESET_TIME) {
                   robotConfig.shooter.stopAll();
                   follower.followPath(path6);
                   setPathState(9);
               }
               break;
           case 9:
               // Wait for Path 6 to complete
               if (!follower.isBusy()) {
                   follower.followPath(path7);
                   setPathState(10);
               }
               break;
           case 10:
               // Wait for Path 7 to complete
               if (!follower.isBusy()) {
                   follower.followPath(path8);
                   setPathState(11);
               }
               break;
           case 11:
               // Wait for Path 8 to complete
               if (!follower.isBusy()) {
                   follower.breakFollowing();
                   waitTimer.resetTimer();
                   setPathState(12);
               }
               break;
           case 12:
               // Shooting Position 4 - Prepare shooter
               robotConfig.shooter.prepareShot(follower.getPose());
               setPathState(121);
               break;
           case 121:
               // Shooting Position 4 - Wait for spin up
               if (waitTimer.getElapsedTimeSeconds() >= SHOOTER_SPIN_UP_TIME) {
                   robotConfig.shooter.shoot();
                   setPathState(122);
               }
               break;
           case 122:
               // Shooting Position 4 - Wait for ball to shoot
               if (waitTimer.getElapsedTimeSeconds() >= SHOOTER_FEED_TIME) {
                   robotConfig.shooter.reset();
                   setPathState(123);
               }
               break;
           case 123:
               // Shooting Position 4 - Final shot, wait for servo reset
               if (waitTimer.getElapsedTimeSeconds() >= SHOOTER_RESET_TIME) {
                   robotConfig.shooter.stopAll();
                   setPathState(-1); // End of autonomous
               }
               break;
       }
   }

   public void setPathState(int pState) {
       pathState = pState;
       pathTimer.resetTimer();
   }

   /**
    * Check if current state is a shooting state
    */
   private boolean isShootingState(int state) {
       return (state >= 2 && state <= 23) ||
              (state >= 5 && state <= 53) ||
              (state >= 8 && state <= 83) ||
              (state >= 12 && state <= 123);
   }

   /**
    * Initialize AprilTag detection with Camera 1
    */
   private void initAprilTag() {
       // Create the AprilTag processor
       aprilTag = new AprilTagProcessor.Builder()
               .setDrawAxes(true)
               .setDrawCubeProjection(true)
               .setDrawTagOutline(true)
               .build();

       // Set decimation for better performance
       aprilTag.setDecimation(2);

       // Create the vision portal with Camera 1
       visionPortal = new VisionPortal.Builder()
               .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
               .addProcessor(aprilTag)
               .build();
   }

   /**
    * Detect AprilTags and return the first detected tag ID
    */
   private void detectAprilTag() {
       List<AprilTagDetection> currentDetections = aprilTag.getDetections();

       if (!currentDetections.isEmpty()) {
           AprilTagDetection detection = currentDetections.get(0);
           detectedAprilTagId = detection.id;

           telemetry.addData("AprilTag Detected", "ID %d", detectedAprilTagId);
           if (detection.metadata != null) {
               telemetry.addData("Tag Name", detection.metadata.name);
           }
       } else {
           telemetry.addData("AprilTag", "None detected");
       }
   }

   @Override
   public void loop() {
       follower.update();

       // Continuously detect AprilTags
       detectAprilTag();

       autonomousPathUpdate();

       telemetry.addData("Path State", pathState);
       telemetry.addData("X", follower.getPose().getX());
       telemetry.addData("Y", follower.getPose().getY());
       telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
       telemetry.addData("Elapsed Time", opmodeTimer.getElapsedTimeSeconds());

       // Show shooter info during shooting sequences
       if (isShootingState(pathState)) {
           double distance = robotConfig.shooter.getDistanceToTarget(follower.getPose());
           double power = robotConfig.shooter.getCalculatedPower(follower.getPose());
           telemetry.addData("", "");
           telemetry.addData("=== SHOOTING ===", "");
           telemetry.addData("Distance", "%.1f inches", distance);
           telemetry.addData("Shooter Power", "%.2f", power);
           telemetry.addData("In Range", robotConfig.shooter.isInRange(follower.getPose()) ? "YES" : "NO");

           // Show which sub-state
           if (pathState % 10 == 1) telemetry.addData("Status", "Spinning up...");
           else if (pathState % 10 == 2) telemetry.addData("Status", "Shooting...");
           else if (pathState % 10 == 3) telemetry.addData("Status", "Resetting...");
           else telemetry.addData("Status", "Preparing...");
       }

       telemetry.update();
   }

   @Override
   public void init() {
       pathTimer = new Timer();
       opmodeTimer = new Timer();
       waitTimer = new Timer();
       opmodeTimer.resetTimer();

       // Initialize AprilTag vision
       initAprilTag();

       // Initialize robot configuration
       robotConfig = new RobotConfig(hardwareMap);

       // Initialize follower
       follower = Constants.createFollower(hardwareMap);
       follower.setStartingPose(startPose);

       // Configure shooter
       robotConfig.shooter.setTargetPosition(SHOOTER_TARGET_X, SHOOTER_TARGET_Y);
       // TODO: Add your calibration values from ShooterTuning here
       // robotConfig.shooter.setCalibration(minDist, minPower, maxDist, maxPower);

       // Build all paths
       buildPaths();

       telemetry.addData("Status", "Initialized");
       telemetry.addData("Camera", "Webcam 1");
       telemetry.addData("Shooter Target", "X: %.1f, Y: %.1f", SHOOTER_TARGET_X, SHOOTER_TARGET_Y);
       telemetry.update();
   }

   @Override
   public void init_loop() {
       // Detect AprilTags during initialization
       detectAprilTag();
       telemetry.update();
   }

   @Override
   public void start() {
       opmodeTimer.resetTimer();
       setPathState(0);
   }

   @Override
   public void stop() {
       robotConfig.stopAllMotors();
       if (visionPortal != null) {
           visionPortal.close();
       }
   }
}
