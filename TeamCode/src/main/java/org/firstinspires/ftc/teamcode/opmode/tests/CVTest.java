package org.firstinspires.ftc.teamcode.opmode.tests;

import static org.firstinspires.ftc.teamcode.config.core.RobotConstants.visionCameraHeight;
import static org.firstinspires.ftc.teamcode.config.core.RobotConstants.visionCameraWidth;
import static org.firstinspires.ftc.teamcode.config.core.RobotConstants.visionExposure;
import static org.firstinspires.ftc.teamcode.config.core.RobotConstants.visionGain;
import static org.firstinspires.ftc.teamcode.config.core.RobotConstants.visionWhiteBalance;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.FocusControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl;
import org.firstinspires.ftc.teamcode.config.core.util.Color;
import org.firstinspires.ftc.teamcode.config.core.vision.Sample;
import org.firstinspires.ftc.teamcode.config.core.vision.Vision;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.concurrent.TimeUnit;

@Config
@TeleOp(group = "Tests", name = "CV Test")
public class CVTest extends OpMode {
    //Talk Volatile vs AtomicReference, final to make sure it is only initialized once, etc.
    private MultipleTelemetry mtelemetry;
    public static boolean cameraOpened;

    public static Color color = Color.YELLOW;

    OpenCvWebcam camera;
    Vision detector;
    Gamepad g1, p1;

    @Override
    public void init() {
        mtelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        detector = new Vision(mtelemetry,color);
        detector.forceOn();
        detector.setNeed(999);
        cameraOpened = false;

        camera.openCameraDeviceAsync(new OpenCvWebcam.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {

                camera.getExposureControl().setMode(ExposureControl.Mode.Manual);
////
                camera.getExposureControl().setExposure(visionExposure, TimeUnit.MILLISECONDS);
//
                camera.getGainControl().setGain(visionGain);
//
                camera.getWhiteBalanceControl().setMode(WhiteBalanceControl.Mode.MANUAL);

                camera.getWhiteBalanceControl().setWhiteBalanceTemperature(visionWhiteBalance);

                camera.getFocusControl().setMode(FocusControl.Mode.Fixed);

                camera.setPipeline(detector);

                camera.startStreaming(visionCameraWidth, visionCameraHeight,
                    OpenCvCameraRotation.UPRIGHT, OpenCvWebcam.StreamFormat.MJPEG);

                FtcDashboard.getInstance().startCameraStream(camera, 0);
                cameraOpened = true;
            }

            @Override
            public void onError(int errorCode) {
                mtelemetry.addData("Camera Error", errorCode);
                mtelemetry.update();
            }
        });

        g1 = new Gamepad();
        p1 = new Gamepad();
    }
        @Override
    public void init_loop()  {
        if (!cameraOpened) {
            return;
        }
        mtelemetry.addData("Detected Rects: ",detector.getSamples().size());
        for (Sample rect : detector.getSamples()) {
            mtelemetry.addLine(String.format("Center (%.1f, %.1f)", rect.getX(), rect.getY()));
            mtelemetry.addLine(String.format("Angle %.2f", rect.getAngle()));
        }
        mtelemetry.addData("exposuremax",camera.getExposureControl().getMaxExposure(TimeUnit.MILLISECONDS));
        mtelemetry.addData("exposuremin",camera.getExposureControl().getMinExposure(TimeUnit.MILLISECONDS));
        mtelemetry.addData("whitebalancemax",camera.getWhiteBalanceControl().getMaxWhiteBalanceTemperature());
        mtelemetry.addData("whitebalancemin",camera.getWhiteBalanceControl().getMinWhiteBalanceTemperature());
        mtelemetry.addData("exposure",camera.getExposureControl().getExposure(TimeUnit.MILLISECONDS));
        mtelemetry.addData("whitebalance",camera.getWhiteBalanceControl().getWhiteBalanceTemperature());
        mtelemetry.addData("gain",camera.getGainControl().getGain());
        mtelemetry.update();
    }
    public void loop() {
        p1.copy(g1);
        g1.copy(gamepad1);
        mtelemetry.addData("exposure",visionExposure);
        mtelemetry.addData("whitebalance",visionWhiteBalance);
        if (color == Color.YELLOW)mtelemetry.addLine("yellow");
        if (color == Color.RED)mtelemetry.addLine("red");
        if (color == Color.BLUE)mtelemetry.addLine("blue");
        if (g1.b && !p1.b) {
            visionExposure += 10;
        }
        if (g1.x && !p1.x) {
            visionExposure -= 10;
        }
        if (g1.a && !p1.a) {
            if (color == Color.YELLOW)color = color.RED;
            else if (color == Color.RED)color = color.BLUE;
            else if (color == Color.BLUE)color = color.YELLOW;
        }
        if (g1.dpad_left && !p1.dpad_left) {
            visionWhiteBalance -= 100;
        }
        if (g1.dpad_right && !p1.dpad_right) {
            visionWhiteBalance += 100;
        }
    }
}