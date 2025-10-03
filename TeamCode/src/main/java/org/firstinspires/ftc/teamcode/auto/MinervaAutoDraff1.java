package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.paths.PathChain;
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

   @Override
   public void init() {
       // Initialize robot configuration (motors, servos, etc.)
       robotConfig = new RobotConfig(hardwareMap);

       // Initialize Pedro Pathing follower
       follower = Constants.createFollower(hardwareMap);
       follower.setStartingPose(startPose);

       // Build path 1
       path1 = follower.pathBuilder()
           .addPath(new BezierLine(new Pose(56.000, 8.000), new Pose(59.344, 23.397)))
           .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(110))
           .build();

       // Build path 2 (two curves combined)
       path2 = follower.pathBuilder()
           .addPath(
               new BezierCurve(
                   new Pose(59.344, 23.397),
                   new Pose(59.344, 39.775),
                   new Pose(18.718, 35.521)
               )
           )
           .setLinearHeadingInterpolation(Math.toRadians(110), Math.toRadians(180))
           .addPath(
               new BezierCurve(
                   new Pose(18.718, 35.521),
                   new Pose(58.706, 40.839),
                   new Pose(59.770, 23.397)
               )
           )
           .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(110))
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
                   pathState = 2;
               }
               break;
           case 2:
               // Execute shoot function at 0.75 power
               robotConfig.shooter.shoot(0.75);
               pathState = 3;
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
                   pathState = 5;
               }
               break;
           case 5:
               // Execute shoot function again at 0.75 power
               robotConfig.shooter.shoot(0.75);
               pathState = 6;
               break;
           case 6:
               // Done - stop all motors
               robotConfig.shooter.stopAll();
               pathState = -1;
               break;
       }

       // Display telemetry
       telemetry.addData("Path State", pathState);
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
