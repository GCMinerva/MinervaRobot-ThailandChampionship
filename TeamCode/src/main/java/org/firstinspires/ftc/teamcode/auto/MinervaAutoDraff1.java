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
import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Minerva Auto Draff 1", group = "Auto")
public class MinervaAutoDraff1 extends OpMode {

   private Follower follower;
   private Timer pathTimer, opmodeTimer, waitTimer;
   private int pathState;
   private RobotConfig robotConfig;


   // Configuration constants - change these to customize behavior
   private static final double START_X = 56.000;
   private static final double START_Y = 8.000;
   private static final double START_HEADING = 90; // degrees

   // Shooter configuration
   private static final double SHOOTER_TARGET_X = 11.0;  // Shooting target X position
   private static final double SHOOTER_TARGET_Y = 135.0; // Shooting target Y position

   // Ball shooter timing (servo up -> down -> outtake -> servo up cycle)
   private static final double SERVO_UP_TIME = 0.5;      // Servo at 180 degrees
   private static final double SERVO_DOWN_TIME = 0.5;    // Servo moving down
   private static final double OUTTAKE_TIME = 1.5;       // Outtake active time
   private static final double SERVO_RESET_TIME = 0.5;   // Servo moving back up

   // Ball shooter cycles per shooting position
   private static final int CYCLES_PATH1 = 2;  // 2 cycles after path 1
   private static final int CYCLES_PATH3 = 3;  // 3 cycles after path 3
   private static final int CYCLES_PATH5 = 3;  // 3 cycles after path 5
   private static final int CYCLES_PATH8 = 3;  // 3 cycles after path 8

   // Shooter power settings for each position
   private static final double POWER_PATH1 = 0.75;
   private static final double POWER_PATH3 = 0.7;
   private static final double POWER_PATH5 = 0.65;
   private static final double POWER_PATH8 = 0.6;

   private int currentCycle = 0;  // Track current shooting cycle

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
                   currentCycle = 0;
                   robotConfig.shooter.startBallShooter(POWER_PATH1);
                   setPathState(2);
               }
               break;
           // === Ball Shooter Sequence Path 1 (2 cycles at 0.75 power) ===
           case 2:
               // Start cycle: Servo up to 180
               robotConfig.shooter.servoUp();
               setPathState(21);
               break;
           case 21:
               // Wait for servo up
               if (waitTimer.getElapsedTimeSeconds() >= SERVO_UP_TIME) {
                   robotConfig.shooter.servoDown();
                   setPathState(22);
               }
               break;
           case 22:
               // Wait for servo down, then activate outtake
               if (waitTimer.getElapsedTimeSeconds() >= SERVO_UP_TIME + SERVO_DOWN_TIME) {
                   robotConfig.shooter.activateOuttake();
                   setPathState(23);
               }
               break;
           case 23:
               // Wait for outtake time
               if (waitTimer.getElapsedTimeSeconds() >= SERVO_UP_TIME + SERVO_DOWN_TIME + OUTTAKE_TIME) {
                   robotConfig.shooter.deactivateOuttake();
                   robotConfig.shooter.servoUp();
                   setPathState(24);
               }
               break;
           case 24:
               // Wait for servo reset, check if more cycles needed
               if (waitTimer.getElapsedTimeSeconds() >= SERVO_UP_TIME + SERVO_DOWN_TIME + OUTTAKE_TIME + SERVO_RESET_TIME) {
                   currentCycle++;
                   if (currentCycle < CYCLES_PATH1) {
                       waitTimer.resetTimer();
                       setPathState(2); // Repeat cycle
                   } else {
                       robotConfig.shooter.stopAll();
                       follower.followPath(path2);
                       setPathState(3);
                   }
               }
               break;
           case 3:
               // Path 2 - Activate outtake
               robotConfig.shooter.activateOuttake();
               setPathState(31);
               break;
           case 31:
               // Wait for Path 2 to complete
               if (!follower.isBusy()) {
                   robotConfig.shooter.deactivateOuttake();
                   follower.followPath(path3);
                   setPathState(4);
               }
               break;
           case 4:
               // Wait for Path 3 to complete
               if (!follower.isBusy()) {
                   follower.breakFollowing();
                   waitTimer.resetTimer();
                   currentCycle = 0;
                   robotConfig.shooter.startBallShooter(POWER_PATH3);
                   setPathState(5);
               }
               break;
           // === Ball Shooter Sequence Path 3 (3 cycles at 0.7 power) ===
           case 5:
               // Start cycle: Servo up to 180
               robotConfig.shooter.servoUp();
               setPathState(51);
               break;
           case 51:
               // Wait for servo up
               if (waitTimer.getElapsedTimeSeconds() >= SERVO_UP_TIME) {
                   robotConfig.shooter.servoDown();
                   setPathState(52);
               }
               break;
           case 52:
               // Wait for servo down, then activate outtake
               if (waitTimer.getElapsedTimeSeconds() >= SERVO_UP_TIME + SERVO_DOWN_TIME) {
                   robotConfig.shooter.activateOuttake();
                   setPathState(53);
               }
               break;
           case 53:
               // Wait for outtake time
               if (waitTimer.getElapsedTimeSeconds() >= SERVO_UP_TIME + SERVO_DOWN_TIME + OUTTAKE_TIME) {
                   robotConfig.shooter.deactivateOuttake();
                   robotConfig.shooter.servoUp();
                   setPathState(54);
               }
               break;
           case 54:
               // Wait for servo reset, check if more cycles needed
               if (waitTimer.getElapsedTimeSeconds() >= SERVO_UP_TIME + SERVO_DOWN_TIME + OUTTAKE_TIME + SERVO_RESET_TIME) {
                   currentCycle++;
                   if (currentCycle < CYCLES_PATH3) {
                       waitTimer.resetTimer();
                       setPathState(5); // Repeat cycle
                   } else {
                       robotConfig.shooter.stopAll();
                       follower.followPath(path4);
                       setPathState(6);
                   }
               }
               break;
           case 6:
               // Path 4 - Activate outtake
               robotConfig.shooter.activateOuttake();
               setPathState(61);
               break;
           case 61:
               // Wait for Path 4 to complete
               if (!follower.isBusy()) {
                   robotConfig.shooter.deactivateOuttake();
                   follower.followPath(path5);
                   setPathState(7);
               }
               break;
           case 7:
               // Wait for Path 5 to complete
               if (!follower.isBusy()) {
                   follower.breakFollowing();
                   waitTimer.resetTimer();
                   currentCycle = 0;
                   robotConfig.shooter.startBallShooter(POWER_PATH5);
                   setPathState(8);
               }
               break;
           // === Ball Shooter Sequence Path 5 (3 cycles at 0.65 power) ===
           case 8:
               // Start cycle: Servo up to 180
               robotConfig.shooter.servoUp();
               setPathState(81);
               break;
           case 81:
               // Wait for servo up
               if (waitTimer.getElapsedTimeSeconds() >= SERVO_UP_TIME) {
                   robotConfig.shooter.servoDown();
                   setPathState(82);
               }
               break;
           case 82:
               // Wait for servo down, then activate outtake
               if (waitTimer.getElapsedTimeSeconds() >= SERVO_UP_TIME + SERVO_DOWN_TIME) {
                   robotConfig.shooter.activateOuttake();
                   setPathState(83);
               }
               break;
           case 83:
               // Wait for outtake time
               if (waitTimer.getElapsedTimeSeconds() >= SERVO_UP_TIME + SERVO_DOWN_TIME + OUTTAKE_TIME) {
                   robotConfig.shooter.deactivateOuttake();
                   robotConfig.shooter.servoUp();
                   setPathState(84);
               }
               break;
           case 84:
               // Wait for servo reset, check if more cycles needed
               if (waitTimer.getElapsedTimeSeconds() >= SERVO_UP_TIME + SERVO_DOWN_TIME + OUTTAKE_TIME + SERVO_RESET_TIME) {
                   currentCycle++;
                   if (currentCycle < CYCLES_PATH5) {
                       waitTimer.resetTimer();
                       setPathState(8); // Repeat cycle
                   } else {
                       robotConfig.shooter.stopAll();
                       follower.followPath(path6);
                       setPathState(9);
                   }
               }
               break;
           case 9:
               // Path 6 - Wait for completion
               if (!follower.isBusy()) {
                   follower.followPath(path7);
                   robotConfig.shooter.activateOuttake();
                   setPathState(10);
               }
               break;
           case 10:
               // Wait for Path 7 to complete
               if (!follower.isBusy()) {
                   robotConfig.shooter.deactivateOuttake();
                   follower.followPath(path8);
                   setPathState(11);
               }
               break;
           case 11:
               // Wait for Path 8 to complete
               if (!follower.isBusy()) {
                   follower.breakFollowing();
                   waitTimer.resetTimer();
                   currentCycle = 0;
                   robotConfig.shooter.startBallShooter(POWER_PATH8);
                   setPathState(12);
               }
               break;
           // === Ball Shooter Sequence Path 8 (3 cycles at 0.6 power) ===
           case 12:
               // Start cycle: Servo up to 180
               robotConfig.shooter.servoUp();
               setPathState(121);
               break;
           case 121:
               // Wait for servo up
               if (waitTimer.getElapsedTimeSeconds() >= SERVO_UP_TIME) {
                   robotConfig.shooter.servoDown();
                   setPathState(122);
               }
               break;
           case 122:
               // Wait for servo down, then activate outtake
               if (waitTimer.getElapsedTimeSeconds() >= SERVO_UP_TIME + SERVO_DOWN_TIME) {
                   robotConfig.shooter.activateOuttake();
                   setPathState(123);
               }
               break;
           case 123:
               // Wait for outtake time
               if (waitTimer.getElapsedTimeSeconds() >= SERVO_UP_TIME + SERVO_DOWN_TIME + OUTTAKE_TIME) {
                   robotConfig.shooter.deactivateOuttake();
                   robotConfig.shooter.servoUp();
                   setPathState(124);
               }
               break;
           case 124:
               // Wait for servo reset, check if more cycles needed
               if (waitTimer.getElapsedTimeSeconds() >= SERVO_UP_TIME + SERVO_DOWN_TIME + OUTTAKE_TIME + SERVO_RESET_TIME) {
                   currentCycle++;
                   if (currentCycle < CYCLES_PATH8) {
                       waitTimer.resetTimer();
                       setPathState(12); // Repeat cycle
                   } else {
                       robotConfig.shooter.stopAll();
                       setPathState(-1); // End of autonomous
                   }
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
       return (state >= 2 && state <= 24) ||
              (state >= 5 && state <= 54) ||
              (state >= 8 && state <= 84) ||
              (state >= 12 && state <= 124);
   }


   @Override
   public void loop() {
       follower.update();
       autonomousPathUpdate();

       telemetry.addData("Path State", pathState);
       telemetry.addData("X", follower.getPose().getX());
       telemetry.addData("Y", follower.getPose().getY());
       telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
       telemetry.addData("Elapsed Time", opmodeTimer.getElapsedTimeSeconds());

       // Show shooter info during shooting sequences
       if (isShootingState(pathState)) {
           telemetry.addData("", "");
           telemetry.addData("=== BALL SHOOTER ===", "");
           telemetry.addData("Cycle", "%d / %d", currentCycle + 1, getCyclesForCurrentPath());
           telemetry.addData("Shooter Power", "%.2f", getShooterPowerForCurrentPath());

           // Show which sub-state
           int subState = pathState % 10;
           if (subState == 0 || subState == 1) {
               telemetry.addData("Status", "Servo Up");
           } else if (subState == 2) {
               telemetry.addData("Status", "Servo Down");
           } else if (subState == 3) {
               telemetry.addData("Status", "Outtake Active");
           } else if (subState == 4) {
               telemetry.addData("Status", "Servo Reset");
           }
       }

       telemetry.update();
   }

   /**
    * Get number of cycles for current shooting position
    */
   private int getCyclesForCurrentPath() {
       if (pathState >= 2 && pathState <= 24) return CYCLES_PATH1;
       if (pathState >= 5 && pathState <= 54) return CYCLES_PATH3;
       if (pathState >= 8 && pathState <= 84) return CYCLES_PATH5;
       if (pathState >= 12 && pathState <= 124) return CYCLES_PATH8;
       return 0;
   }

   /**
    * Get shooter power for current shooting position
    */
   private double getShooterPowerForCurrentPath() {
       if (pathState >= 2 && pathState <= 24) return POWER_PATH1;
       if (pathState >= 5 && pathState <= 54) return POWER_PATH3;
       if (pathState >= 8 && pathState <= 84) return POWER_PATH5;
       if (pathState >= 12 && pathState <= 124) return POWER_PATH8;
       return 0.0;
   }

   @Override
   public void init() {
       pathTimer = new Timer();
       opmodeTimer = new Timer();
       waitTimer = new Timer();
       opmodeTimer.resetTimer();

       // Initialize robot configuration
       robotConfig = new RobotConfig(hardwareMap);

       // Initialize follower
       follower = Constants.createFollower(hardwareMap);
       follower.setStartingPose(startPose);

       // Configure shooter
       robotConfig.shooter.setTargetPosition(SHOOTER_TARGET_X, SHOOTER_TARGET_Y);

       // Build all paths
       buildPaths();

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
       opmodeTimer.resetTimer();
       setPathState(0);
   }

   @Override
   public void stop() {
       robotConfig.stopAllMotors();
   }
}
