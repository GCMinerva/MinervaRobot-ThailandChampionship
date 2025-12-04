package org.firstinspires.ftc.teamcode.archived.subsystems;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/**
 * Dedicated AprilTag tracker for autonomous turret aiming.
 * Shares VisionPortal with AprilTagSubsystem to avoid multiple portal error.
 * Tracks goal tags (24 for Red, 20 for Blue) during autonomous.
 */
public class AutoTurretTracker {

    private final AprilTagSubsystem aprilTagSubsystem;

    private int targetTagId = 24; // Default: Red goal
    private double bearingDegrees = 0.0;
    private boolean goalDetected = false;

    /**
     * Create tracker that shares VisionPortal with AprilTagSubsystem
     * 
     * @param aprilTagSubsystem The existing AprilTagSubsystem to share camera with
     */
    public AutoTurretTracker(AprilTagSubsystem aprilTagSubsystem) {
        this.aprilTagSubsystem = aprilTagSubsystem;
    }

    /**
     * Update AprilTag detection and calculate bearing to goal
     */
    public void update() {
        // Get detections from shared AprilTagSubsystem
        List<AprilTagDetection> detections = aprilTagSubsystem.getDetections();
        goalDetected = false;

        if (detections != null && !detections.isEmpty()) {
            for (AprilTagDetection tag : detections) {
                if (tag.id == targetTagId) {
                    goalDetected = true;

                    // Calculate bearing from AprilTag
                    if (tag.ftcPose != null) {
                        bearingDegrees = tag.ftcPose.bearing;
                        // Invert for upside-down camera
                        bearingDegrees = -bearingDegrees;
                    }
                    break;
                }
            }
        }
    }

    public void setTargetTagId(int tagId) {
        this.targetTagId = tagId;
    }

    public boolean isGoalDetected() {
        return goalDetected;
    }

    public double getBearingDegrees() {
        return bearingDegrees;
    }

    public void stop() {
        // Don't close VisionPortal - it's owned by AprilTagSubsystem
    }
}
