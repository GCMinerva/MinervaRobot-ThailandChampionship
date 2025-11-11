package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

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

    public AprilTagSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        aprilTag = new AprilTagProcessor.Builder().build();
        aprilTag.setDecimation(2);

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
    }

    /** Call during INIT loop. Stores valid tag ID (21–23) only. */
    public void update() {
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

    public void stop() {
        if (visionPortal != null) visionPortal.close();
    }
}
