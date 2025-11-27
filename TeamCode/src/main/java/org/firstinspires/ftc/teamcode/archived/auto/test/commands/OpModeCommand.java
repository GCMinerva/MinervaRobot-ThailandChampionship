package org.firstinspires.ftc.teamcode.archived.auto.test.commands;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.Subsystem;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * Base class for command-based OpModes
 * Integrates FTCLib CommandScheduler with FTC OpMode lifecycle
 */
public abstract class OpModeCommand extends OpMode {

    /**
     * Reset the command scheduler
     */
    public void reset() {
        CommandScheduler.getInstance().reset();
    }

    /**
     * Run the command scheduler (processes all scheduled commands)
     */
    public void run() {
        CommandScheduler.getInstance().run();
    }

    /**
     * Schedule commands to run
     */
    public void schedule(Command... commands) {
        CommandScheduler.getInstance().schedule(commands);
    }

    /**
     * Register subsystems with the scheduler
     */
    public void register(Subsystem... subsystems) {
        CommandScheduler.getInstance().registerSubsystem(subsystems);
    }

    @Override
    public void init() {
        // Ensure gamepads are initialized to prevent NullPointerException during stop()
        // This is especially important for Autonomous OpModes
        if (gamepad1 == null) {
            gamepad1 = new com.qualcomm.robotcore.hardware.Gamepad();
        }
        if (gamepad2 == null) {
            gamepad2 = new com.qualcomm.robotcore.hardware.Gamepad();
        }
        initialize();
    }

    @Override
    public void loop() {
        run();
        iter();
    }

    @Override
    public void stop() {
        try {
            // Cancel all commands before resetting to prevent "stuck in stop()" errors
            CommandScheduler.getInstance().cancelAll();
            end();
            reset();
        } catch (Exception e) {
            // Catch any exceptions during stop to prevent "stuck in stop()" errors
            // This commonly happens with NullPointerException on gamepad.resetEdgeDetection()
            telemetry.addData("Stop Error", e.getMessage());
            telemetry.update();
        }
    }

    /**
     * Initialize your robot and schedule commands here
     */
    public abstract void initialize();

    /**
     * Called every loop iteration (use for telemetry updates, etc.)
     */
    public abstract void iter();

    /**
     * Called when the OpMode stops
     */
    public abstract void end();
}
