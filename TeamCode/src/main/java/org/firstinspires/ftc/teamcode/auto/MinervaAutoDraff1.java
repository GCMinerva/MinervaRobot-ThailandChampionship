package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathConstraints;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Minerva Auto Draff 1", group = "Auto")
public class MinervaAutoDraff1 extends OpMode {

   private Follower follower;
   private RobotConfig robotConfig;

   // Start position
   private static final double START_X = 56.000;
   private static final double START_Y = 8.000;
   private static final double START_HEADING = 90; // degrees

   private final Pose startPose = new Pose(START_X, START_Y, Math.toRadians(START_HEADING));

   private PathChain path1;
   private PathChain path2;
   private int pathState = 0;
   private Timer shootTimer;
   private int shootCycle = 0;

   @Override
   public void init() {
       // Initialize robot configuration (motors, servos, etc.)
       robotConfig = new RobotConfig(hardwareMap);

       // Initialize Pedro Pathing follower
       follower = Constants.createFollower(hardwareMap);
       follower.setStartingPose(startPose);

       // Initialize timer
       shootTimer = new Timer();

       // Build path 1
       path1 = follower.pathBuilder()
           .addPath(new BezierLine(new Pose(56.000, 8.000), new Pose(59.344, 23.397)))
           .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(100))
           .build();

       path2 = follower.pathBuilder()
           .addPath(
               new BezierCurve(
                   new Pose(59.344, 23.397),
                   new Pose(59.344, 39.775),
                   new Pose(18.718, 35.521)
               )
           )
           .setLinearHeadingInterpolation(Math.toRadians(100), Math.toRadians(180))
           .addPath(
               new BezierCurve(
                   new Pose(18.718, 35.521),
                   new Pose(58.706, 40.839),
                   new Pose(59.770, 23.397)
               )
           )
           .setConstantHeadingInterpolation(Math.toRadians(180))
           .build();

       telemetry.addData("Status", "Initialized");
       telemetry.addData("Start Position", "X: %.1f, Y: %.1f, Heading: %.0f°", START_X, START_Y, START_HEADING);
       telemetry.update();
   }

   @Override
   public void init_loop() {
       telemetry.addData("Status", "Ready to Start");
       telemetry.update();
   }

   @Override
   public void start() {
       // Start the path
       follower.followPath(path1);
       pathState = 1;
   }

   @Override
   public void loop() {
       // Update follower
       follower.update();

       // State machine
       switch (pathState) {
           case 1:
               // Wait for path 1 to complete
               if (!follower.isBusy()) {
                   follower.breakFollowing();
                   // Start shooting sequence at 0.85 power
                   robotConfig.shooter.startShootingSequence(0.85);
                   shootCycle = 0;
                   shootTimer.resetTimer();
                   pathState = 2;
               }
               break;

           // === SHOOTING SEQUENCE 1 (3 cycles) ===
           case 2:
               // Servo up and stop outtake
               robotConfig.shooter.servoUp();
               robotConfig.shooter.deactivateOuttake();
               shootTimer.resetTimer();
               pathState = 21;
               break;
           case 21:
               // Wait 0.1 second then servo down
               if (shootTimer.getElapsedTimeSeconds() >= 0.1) {
                   robotConfig.shooter.servoDown();
                   shootTimer.resetTimer();
                   pathState = 22;
               }
               break;
           case 22:
               // Wait 2 seconds
               if (shootTimer.getElapsedTimeSeconds() >= 2.0) {
                   shootTimer.resetTimer();
                   pathState = 23;
               }
               break;
           case 23:
               // Activate outtake
               robotConfig.shooter.activateOuttake();
               shootTimer.resetTimer();
               pathState = 24;
               break;
           case 24:
               // Wait 1 second for outtake, then servo up again
               if (shootTimer.getElapsedTimeSeconds() >= 1.0) {
                   robotConfig.shooter.deactivateOuttake();
                   robotConfig.shooter.servoUp();
                   shootTimer.resetTimer();
                   pathState = 25;
               }
               break;
           case 25:
               // Small delay before next cycle
               if (shootTimer.getElapsedTimeSeconds() >= 0.1) {
                   shootCycle++;

                   if (shootCycle < 3) {
                       shootTimer.resetTimer();
                       pathState = 21; // Repeat from servo down
                   } else {
                       pathState = 3; // Done shooting
                   }
               }
               break;

           case 3:
               // Stop shooter motors and start path 2
               robotConfig.shooter.stopAll();
               follower.followPath(path2);
               pathState = 4;
               break;
           case 4:
               // Wait for path 2 to complete
               if (!follower.isBusy()) {
                   follower.breakFollowing();
                   // Start shooting sequence again at 0.85 power
                   robotConfig.shooter.startShootingSequence(0.85);
                   shootCycle = 0;
                   shootTimer.resetTimer();
                   pathState = 5;
               }
               break;

           // === SHOOTING SEQUENCE 2 (3 cycles) ===
           case 5:
               // Servo up and stop outtake
               robotConfig.shooter.servoUp();
               robotConfig.shooter.deactivateOuttake();
               shootTimer.resetTimer();
               pathState = 51;
               break;
           case 51:
               // Wait 0.1 second then servo down
               if (shootTimer.getElapsedTimeSeconds() >= 0.1) {
                   robotConfig.shooter.servoDown();
                   shootTimer.resetTimer();
                   pathState = 52;
               }
               break;
           case 52:
               // Wait 2 seconds
               if (shootTimer.getElapsedTimeSeconds() >= 2.0) {
                   shootTimer.resetTimer();
                   pathState = 53;
               }
               break;
           case 53:
               // Activate outtake
               robotConfig.shooter.activateOuttake();
               shootTimer.resetTimer();
               pathState = 54;
               break;
           case 54:
               // Wait 1 second for outtake, then servo up again
               if (shootTimer.getElapsedTimeSeconds() >= 1.0) {
                   robotConfig.shooter.deactivateOuttake();
                   robotConfig.shooter.servoUp();
                   shootTimer.resetTimer();
                   pathState = 55;
               }
               break;
           case 55:
               // Small delay before next cycle
               if (shootTimer.getElapsedTimeSeconds() >= 0.1) {
                   shootCycle++;

                   if (shootCycle < 3) {
                       shootTimer.resetTimer();
                       pathState = 51; // Repeat from servo down
                   } else {
                       pathState = 6; // Done shooting
                   }
               }
               break;

           case 6:
               // Done - stop all motors
               robotConfig.shooter.stopAll();
               pathState = -1;
               break;
       }

       // Display telemetry
       telemetry.addData("Path State", pathState);
       telemetry.addData("Shoot Cycle", shootCycle);
       telemetry.addData("X", follower.getPose().getX());
       telemetry.addData("Y", follower.getPose().getY());
       telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
       telemetry.update();
   }

   @Override
   public void stop() {
       robotConfig.stopAllMotors();
   }
}
