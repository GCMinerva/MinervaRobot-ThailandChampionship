package org.firstinspires.ftc.teamcode.archived.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.archived.util.GlobalData;

import java.util.List;
import java.util.concurrent.TimeUnit;

/**
 * AprilTag subsystem that reads tags during INIT.
 * Accepts only IDs 21, 22, or 23.
 * If multiple valid tags appear, ignores all until only one remains.
 */
public class AprilTagSubsystem {

    private static final boolean USE_WEBCAM = true;
    private final Telemetry telemetry;
    private final AprilTagProcessor aprilTag;
    private final VisionPortal visionPortal;

    private AprilTagDetection lastDetection = null;
    private int lastTagId = -1;
    private boolean exposureSet = false; // Track if exposure has been configured

    public AprilTagSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        aprilTag = new AprilTagProcessor.Builder().build();
        aprilTag.setDecimation(1); // Reduced from 2 for better detection range

        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }

        // Don't set exposure here - it will block! Set it in first update() call
    }

    /**
     * Set manual camera exposure for better AprilTag detection (non-blocking)
     */
    private void setManualExposureIfReady() {
        if (exposureSet)
            return; // Already set

        // Only set if camera is ready (don't block)
        if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            try {
                ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
                if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                    exposureControl.setMode(ExposureControl.Mode.Manual);
                }
                exposureControl.setExposure(6, TimeUnit.MILLISECONDS);

                GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
                gainControl.setGain(250);

                exposureSet = true; // Mark as configured
            } catch (Exception e) {
                // Ignore errors, will try again next update
            }
        }
    }

    /** Call during INIT loop. Stores valid tag ID (21–23) only. */
    public void update() {
        // Set exposure on first update when camera is ready (non-blocking)
        setManualExposureIfReady();
        List<AprilTagDetection> detections = aprilTag.getDetections();
        int validCount = 0;
        AprilTagDetection validTag = null;

        if (detections != null && !detections.isEmpty()) {
            for (AprilTagDetection tag : detections) {
                if (tag.id == 21 || tag.id == 22 || tag.id == 23) {
                    validCount++;
                    validTag = tag;
                }
            }
        }

        if (validCount == 1) { // exactly one valid tag visible
            lastDetection = validTag;
            lastTagId = validTag.id;
            GlobalData.detectedTagId = lastTagId; // Write to GlobalData
            telemetry.addData("✅ Detected Tag", lastTagId);
        } else if (validCount > 1) {
            telemetry.addData("⚠ Multiple valid tags", validCount);
        } else {
            telemetry.addData("No valid tag", "");
        }
        telemetry.update();
    }

    public int getLastTagId() {
        return lastTagId;
    }

    /**
     * Get current AprilTag detections for sharing with AutoTurretTracker
     */
    public List<AprilTagDetection> getDetections() {
        return aprilTag.getDetections();
    }

    public void stop() {
        if (visionPortal != null)
            visionPortal.close();
    }
}
