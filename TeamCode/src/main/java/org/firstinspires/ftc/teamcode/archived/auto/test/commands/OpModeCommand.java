package org.firstinspires.ftc.teamcode.archived.auto.test.commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.Subsystem;
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
        initialize();
    }

    @Override
    public void loop() {
        run();
        iter();
    }

    @Override
    public void stop() {
        reset();
        end();
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
