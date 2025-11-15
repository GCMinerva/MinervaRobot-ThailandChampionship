package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.vision.ArtifactCirclePipeline;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.ArrayList;

@TeleOp(name = "Artifact Circle Detection", group = "Vision")
public class ArtifactCircleDetectionOpMode extends LinearOpMode {

    private VisionPortal visionPortal;
    private ArtifactCirclePipeline artifactDetector;
    private FtcDashboard dashboard;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize FTC Dashboard
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        // Initialize artifact detector
        artifactDetector = new ArtifactCirclePipeline();

        // Initialize vision portal with camera streaming to dashboard
        visionPortal = new VisionPortal.Builder()
            .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
            .addProcessor(artifactDetector)
            .build();

        // Start streaming camera to dashboard using VisionPortal
        FtcDashboard.getInstance().startCameraStream(visionPortal, 0);

        telemetry.addLine("Artifact Circle Detection Initialized");
        telemetry.addLine("Camera streaming to FTC Dashboard");
        telemetry.addLine("Access dashboard at: http://192.168.43.1:8080/dash");
        telemetry.addLine("Press Play to start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Get all detected artifacts
            ArrayList<ArtifactCirclePipeline.Artifact> artifacts = artifactDetector.getFoundArtifacts();

            // Display telemetry
            telemetry.addData("Total Artifacts", artifacts != null ? artifacts.size() : 0);
            telemetry.addLine();

            if (artifacts != null && !artifacts.isEmpty()) {
                // Count by color
                int purpleCount = 0;
                int greenCount = 0;

                for (ArtifactCirclePipeline.Artifact artifact : artifacts) {
                    if (artifact.color == ArtifactCirclePipeline.Artifact.Color.PURPLE) {
                        purpleCount++;
                    } else if (artifact.color == ArtifactCirclePipeline.Artifact.Color.GREEN) {
                        greenCount++;
                    }
                }

                telemetry.addData("Purple Artifacts", purpleCount);
                telemetry.addData("Green Artifacts", greenCount);
                telemetry.addLine();

                // Display individual artifacts (limit to first 5)
                for (int i = 0; i < artifacts.size() && i < 5; i++) {
                    ArtifactCirclePipeline.Artifact artifact = artifacts.get(i);
                    telemetry.addLine(String.format("%d. %s Circle",
                        i + 1,
                        artifact.color.name()));
                    telemetry.addLine(String.format("   Center: (%.1f, %.1f)",
                        artifact.center.x,
                        artifact.center.y));
                    telemetry.addLine(String.format("   Radius: %.1f px",
                        artifact.radius));
                }

                // Find closest artifact to center of screen (assuming 640x480)
                ArtifactCirclePipeline.Artifact closest = findClosestToCenter(artifacts, 320, 240);
                if (closest != null) {
                    telemetry.addLine();
                    telemetry.addData("Closest Artifact", closest.color.name());
                    telemetry.addData("Distance from Center",
                        String.format("%.1f px", distanceFromPoint(closest, 320, 240)));
                }
            } else {
                telemetry.addLine("No artifacts detected");
            }

            telemetry.addLine();
            telemetry.addLine("View live camera feed on FTC Dashboard");
            telemetry.addLine("Dashboard URL: http://192.168.43.1:8080/dash");

            telemetry.update();
            sleep(50);
        }

        // Cleanup
        dashboard.stopCameraStream();
        visionPortal.close();
    }

    private ArtifactCirclePipeline.Artifact findClosestToCenter(
            ArrayList<ArtifactCirclePipeline.Artifact> artifacts,
            double centerX,
            double centerY) {

        ArtifactCirclePipeline.Artifact closest = null;
        double minDistance = Double.MAX_VALUE;

        for (ArtifactCirclePipeline.Artifact artifact : artifacts) {
            double distance = distanceFromPoint(artifact, centerX, centerY);
            if (distance < minDistance) {
                minDistance = distance;
                closest = artifact;
            }
        }

        return closest;
    }

    private double distanceFromPoint(
            ArtifactCirclePipeline.Artifact artifact,
            double x,
            double y) {

        double dx = artifact.center.x - x;
        double dy = artifact.center.y - y;
        return Math.sqrt(dx * dx + dy * dy);
    }
}
