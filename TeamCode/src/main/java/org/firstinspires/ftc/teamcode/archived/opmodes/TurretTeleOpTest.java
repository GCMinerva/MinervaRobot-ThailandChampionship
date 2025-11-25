package org.firstinspires.ftc.teamcode.archived.opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.archived.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * Turret TeleOp Test,
 * 
 * Controls:
 * - Gamepad1 Right Stick Y: Set target turret angle manually (-90° to +90°)
 * - Gamepad1 A: Reset turret to center (0°)
 * 
 * Goal Position: Blue goal (0, 144) - FIXED (same as autonomous)
 * 
 * Telemetry shows:
 * - Robot position and heading (from odometry)
 * - Turret current/target angles
 * - Alignment status
 * - Encoder position (ticks)
 */
@TeleOp(name = "Turret TeleOp Test", group = "Test")
public class TurretTeleOpTest extends LinearOpMode {

    private Follower follower;
    private TurretSubsystem turret;

    // Goal position - BLUE GOAL (same as autonomous)
    private static final double GOAL_X = 0.0;
    private static final double GOAL_Y = 144.0;

    // Manual angle control
    private double manualTargetAngle = 0.0;

    @Override
    public void runOpMode() {
        // === Initialize Pedro Follower ===
        follower = Constants.createFollower(hardwareMap);

        // === Set starting pose (CUSTOMIZE THIS FOR YOUR TEST) ===
        // Example: starting at RB3 position
        Pose startingPose = new Pose(110.5, 134.5, Math.toRadians(90));
        follower.setStartingPose(startingPose);

        telemetry.addLine("=== Turret TeleOp Test ===");
        telemetry.addLine("Starting Pose: " + startingPose);
        telemetry.addLine("Controls:");
        telemetry.addLine("  Right Stick Y: Set turret angle");
        telemetry.addLine("  A: Reset to center");
        telemetry.addLine("Goal: Blue goal (0, 144)");
        telemetry.addLine("Waiting for start...");
        telemetry.update();

        // === Initialize Turret Subsystem ===
        turret = new TurretSubsystem(hardwareMap);
        turret.setGoalPosition(GOAL_X, GOAL_Y);
        turret.setStartingAngle(0.0);  // Position turret straight ahead

        waitForStart();

        while (opModeIsActive()) {
            // === Update odometry ===
            follower.update();
            Pose robotPose = follower.getPose();

            // === Handle Gamepad Input ===
            handleGamepadInput();

            // === Update turret toward blue goal ===
            turret.setGoalPosition(GOAL_X, GOAL_Y);
            turret.update(robotPose);

            // === Display Telemetry ===
            displayTelemetry(robotPose);
        }

        // === Cleanup ===
        turret.stop();
        telemetry.addLine("Test Complete");
        telemetry.update();
    }

    /**
     * Handle gamepad input for turret control
     */
    private void handleGamepadInput() {
        // Right stick Y: Manual angle control (-1.0 to 1.0 → -90° to +90°)
        if (Math.abs(gamepad1.right_stick_y) > 0.1) {
            double targetAngle = -gamepad1.right_stick_y * 90.0;
            turret.setGoalPosition(
                turret.getGoalX() + 100 * Math.cos(Math.toRadians(targetAngle)),
                turret.getGoalY() + 100 * Math.sin(Math.toRadians(targetAngle))
            );
        }

        // A Button: Reset turret to center
        if (gamepad1.a) {
            turret.resetToCenter();
            sleep(200);
        }
    }

    /**
     * Display all relevant telemetry
     */
    private void displayTelemetry(Pose robotPose) {
        telemetry.addLine("=== ROBOT STATE ===");
        telemetry.addData("Robot X", "%.1f in", robotPose.getX());
        telemetry.addData("Robot Y", "%.1f in", robotPose.getY());
        telemetry.addData("Robot Heading", "%.1f°", Math.toDegrees(robotPose.getHeading()));

        telemetry.addLine("=== TURRET STATE ===");
        telemetry.addData("Current Angle", "%.1f°", turret.getCurrentAngle());
        telemetry.addData("Target Angle", "%.1f°", turret.getTargetAngle());
        telemetry.addData("Angle Error", "%.1f°", turret.getAngleError());
        telemetry.addData("Encoder Ticks", turret.turretMotor.getCurrentPosition());
        telemetry.addData("Aligned", turret.isAligned());

        telemetry.addLine("=== GOAL (FIXED) ===");
        telemetry.addData("Goal X", "%.1f in", GOAL_X);
        telemetry.addData("Goal Y", "%.1f in", GOAL_Y);

        telemetry.addLine("=== CONTROLS ===");
        telemetry.addLine("Right Stick Y: Adjust angle");
        telemetry.addLine("A: Reset to center");

        telemetry.update();
    }
}
