package org.firstinspires.ftc.teamcode.archived.auto.test.commands;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.archived.subsystems.AutoTurretTracker;
import org.firstinspires.ftc.teamcode.archived.subsystems.TurretSubsystem;

/**
 * Continuous turret tracking for autonomous using dedicated AutoTurretTracker.
 * Runs in parallel with paths without affecting init detection.
 */
public class AutoTurretTracking extends CommandBase {

    private final AutoTurretTracker tracker;
    private final TurretSubsystem turret;
    private final DcMotor turretMotor;

    // Turret constants
    private static final double TICKS_PER_DEGREE = 6.208;
    private static final double MIN_ANGLE = -90.0;
    private static final double MAX_ANGLE = 90.0;
    private static final double TRACKING_POWER = 0.4;
    private static final double BEARING_DEADBAND = 2.0;

    public AutoTurretTracking(AutoTurretTracker tracker, TurretSubsystem turret, DcMotor turretMotor) {
        this.tracker = tracker;
        this.turret = turret;
        this.turretMotor = turretMotor;
    }

    @Override
    public void execute() {
        // Update AprilTag detection
        tracker.update();

        // If goal is detected, aim turret at it
        if (tracker.isGoalDetected()) {
            double bearing = tracker.getBearingDegrees();

            // Only update if bearing error is significant
            if (Math.abs(bearing) > BEARING_DEADBAND) {
                int currentTicks = turretMotor.getCurrentPosition();
                double currentAngle = currentTicks / TICKS_PER_DEGREE;
                double targetAngle = currentAngle - bearing + 3;

                setTurretAngle(targetAngle);
            }
        }
    }

    private void setTurretAngle(double angleDegrees) {
        double clampedAngle = Math.max(MIN_ANGLE, Math.min(MAX_ANGLE, angleDegrees));
        int targetTicks = (int) (clampedAngle * TICKS_PER_DEGREE);

        if (turretMotor.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
            turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        turretMotor.setTargetPosition(targetTicks);
        turretMotor.setPower(TRACKING_POWER);
    }

    @Override
    public boolean isFinished() {
        return false; // Runs continuously
    }
}
