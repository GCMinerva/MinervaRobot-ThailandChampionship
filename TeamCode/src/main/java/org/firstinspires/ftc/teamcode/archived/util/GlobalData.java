package org.firstinspires.ftc.teamcode.archived.util;

import com.pedropathing.geometry.Pose;

/** Stores global data accessible across OpModes */
public class GlobalData {
    public static int detectedTagId = 21; // universal AprilTag ID

    // Autonomous end pose for Manual OpMode handoff
    public static Pose autonomousEndPose = null;
    public static boolean hasAutonomousRun = false;
}
