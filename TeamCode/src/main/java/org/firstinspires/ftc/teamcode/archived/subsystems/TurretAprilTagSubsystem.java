package org.firstinspires.ftc.teamcode.archived.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;
import java.util.concurrent.TimeUnit;

/**
 * TurretAprilTagSubsystem - AprilTag detection for turret tracking
 * 
 * Designed specifically for turret aiming using AprilTags on goals.
 * Supports upside-down camera mounting and provides bearing/distance data.
 * 
 * Features:
 * - Handles upside-down camera orientation
 * - Calculates bearing angle for turret positioning
 * - Filters detections by tag ID (configurable for Red/Blue goals)
 * - Provides distance and angle data for aiming
 * 
 * Usage:
 * 1. Create subsystem: turretTag = new TurretAprilTagSubsystem(hardwareMap,
 * telemetry)
 * 2. Set target tag ID: turretTag.setTargetTagId(11) // Red goal
 * 3. In loop: turretTag.update()
 * 4. Get bearing: double angle = turretTag.getBearingDegrees()
 * 5. Check if detected: if (turretTag.isTagDetected()) { ... }
 */
public class TurretAprilTagSubsystem {

    private final Telemetry telemetry;
    private final AprilTagProcessor aprilTag;
    private final VisionPortal visionPortal;

    // Camera configuration
    private static final boolean USE_WEBCAM = true;
    private static final boolean CAMERA_UPSIDE_DOWN = true; // Set to true if camera is mounted upside down

    // Target tag configuration
    private int targetTagId = -1; // -1 = accept any tag

    // Detection data
    private AprilTagDetection currentDetection = null;
    private boolean tagDetected = false;
    private double bearingDegrees = 0.0;
    private double distanceInches = 0.0;
    private double elevationDegrees = 0.0;

    // Camera field of view (adjust based on your camera)
    private static final double CAMERA_FOV_HORIZONTAL_DEGREES = 78.0; // Typical webcam FOV

    public TurretAprilTagSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        // Create AprilTag processor WITH camera calibration for accurate pose
        // estimation
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                // Apply camera calibration for accurate 3D pose
                .setLensIntrinsics(
                        org.firstinspires.ftc.teamcode.archived.util.CameraCalibrate.FX,
                        org.firstinspires.ftc.teamcode.archived.util.CameraCalibrate.FY,
                        org.firstinspires.ftc.teamcode.archived.util.CameraCalibrate.CX,
                        org.firstinspires.ftc.teamcode.archived.util.CameraCalibrate.CY)
                .build();

        // Set decimation to 1 for better framerate (1 = no decimation, faster
        // processing)
        aprilTag.setDecimation(1);

        // Build vision portal with balanced settings
        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag)
                    .setCameraResolution(new android.util.Size(320, 240)) // Balanced: 30 FPS, good range
                    .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                    .enableLiveView(true)
                    .setAutoStopLiveView(true)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .setCameraResolution(new android.util.Size(320, 240))
                    .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                    .enableLiveView(true)
                    .setAutoStopLiveView(true)
                    .build();
        }

        // Optional: Set camera exposure and gain for better detection
        setManualExposure(6, 250);
    }

    /**
     * Set target AprilTag ID to track
     * 
     * @param tagId Tag ID to track (e.g., 11 for Red goal, 12 for Blue goal)
     *              Set to -1 to accept any tag
     */
    public void setTargetTagId(int tagId) {
        this.targetTagId = tagId;
    }

    /**
     * Update detection - call this in your loop()
     * Processes AprilTag detections and calculates bearing/distance
     */
    public void update() {
        List<AprilTagDetection> detections = aprilTag.getDetections();

        // Reset detection state
        tagDetected = false;
        currentDetection = null;

        if (detections != null && !detections.isEmpty()) {
            // Find target tag or use first detection if no target set
            for (AprilTagDetection detection : detections) {
                if (targetTagId == -1 || detection.id == targetTagId) {
                    currentDetection = detection;
                    tagDetected = true;

                    // Calculate bearing and distance
                    calculateBearingAndDistance(detection);
                    break;
                }
            }
        }
    }

    /**
     * Calculate bearing angle and distance from AprilTag detection
     * Handles upside-down camera orientation
     */
    private void calculateBearingAndDistance(AprilTagDetection detection) {
        // Get tag center position in image coordinates
        // ftcPose gives us X, Y, Z in inches and bearing/range/elevation
        if (detection.ftcPose != null) {
            // Use FTC's built-in bearing calculation
            bearingDegrees = detection.ftcPose.bearing;
            distanceInches = detection.ftcPose.range;
            elevationDegrees = detection.ftcPose.elevation;

            // If camera is upside down, invert the bearing
            if (CAMERA_UPSIDE_DOWN) {
                bearingDegrees = -bearingDegrees;
            }
        } else {
            // Fallback: calculate from image center position
            // This is less accurate but works if ftcPose is null
            double centerX = detection.center.x;
            double imageWidth = 640; // Typical webcam resolution width

            // Calculate bearing based on position in image
            // Center of image = 0°, left = negative, right = positive
            double pixelOffset = centerX - (imageWidth / 2.0);
            bearingDegrees = (pixelOffset / (imageWidth / 2.0)) * (CAMERA_FOV_HORIZONTAL_DEGREES / 2.0);

            // If camera is upside down, invert the bearing
            if (CAMERA_UPSIDE_DOWN) {
                bearingDegrees = -bearingDegrees;
            }

            // Distance estimation (requires known tag size - less accurate)
            distanceInches = 0.0; // Not available without ftcPose
        }
    }

    /**
     * Check if target tag is currently detected
     * 
     * @return true if tag is visible
     */
    public boolean isTagDetected() {
        return tagDetected;
    }

    /**
     * Get bearing angle to tag in degrees
     * 
     * @return Bearing angle (-90 to +90 degrees)
     *         Negative = tag is to the left
     *         Positive = tag is to the right
     *         0 = tag is straight ahead
     */
    public double getBearingDegrees() {
        return bearingDegrees;
    }

    /**
     * Get distance to tag in inches
     * 
     * @return Distance in inches (0 if not available)
     */
    public double getDistanceInches() {
        return distanceInches;
    }

    /**
     * Get elevation angle to tag in degrees
     * 
     * @return Elevation angle in degrees
     */
    public double getElevationDegrees() {
        return elevationDegrees;
    }

    /**
     * Get detected tag ID
     * 
     * @return Tag ID or -1 if no tag detected
     */
    public int getDetectedTagId() {
        return (currentDetection != null) ? currentDetection.id : -1;
    }

    /**
     * Get current AprilTag detection
     * 
     * @return AprilTagDetection object or null if no tag detected
     */
    public AprilTagDetection getCurrentDetection() {
        return currentDetection;
    }

    /**
     * Set manual camera exposure for better AprilTag detection
     * 
     * @param exposureMS Exposure time in milliseconds (6 is good for indoor)
     * @param gain       Camera gain (250 is good for indoor)
     */
    private void setManualExposure(int exposureMS, int gain) {
        // Wait for vision portal to be ready
        while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            try {
                Thread.sleep(20);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        // Set exposure and gain
        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
            exposureControl.setMode(ExposureControl.Mode.Manual);
            try {
                Thread.sleep(50);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
        exposureControl.setExposure(exposureMS, TimeUnit.MILLISECONDS);

        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        gainControl.setGain(gain);
    }

    /**
     * Stop the vision portal and release camera
     */
    public void stop() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }

    /**
     * Add telemetry data for debugging
     */
    public void addTelemetry() {
        if (tagDetected) {
            telemetry.addData("🎯 Tag Detected", getDetectedTagId());
            telemetry.addData("Bearing", "%.1f°", bearingDegrees);
            telemetry.addData("Distance", "%.1f\"", distanceInches);
            telemetry.addData("Elevation", "%.1f°", elevationDegrees);
        } else {
            telemetry.addData("Tag Status", "No target detected");
            if (targetTagId != -1) {
                telemetry.addData("Looking for Tag", targetTagId);
            }
        }
    }
}
