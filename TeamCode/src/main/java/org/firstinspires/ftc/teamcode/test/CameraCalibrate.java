package org.firstinspires.ftc.teamcode.test;

/**
 * Camera Calibration Data
 * Generated using 3DF Zephyr or MRCAL calibration tool
 *
 * Calibration Pattern Results:
 *
 * Focals (pixels):
 *   Fx: 249.376
 *   Fy: 249.376
 *
 * Optical Center:
 *   Cx: 1062.11
 *   Cy: 460.204
 *
 * Radial Distortion (Brown's Model):
 *   K1: 0.000152435
 *   K2: -8.40784e-05
 *   K3: 1.8523e-06
 *   P1: 0.000314673
 *   P2: 0.000336862
 *
 * Skew: 0
 *
 * Statistics:
 *   Mean Square Reprojection Error: 0.378256 pixels
 *
 * Note: This calibration is specific to the camera, lens, and resolution used.
 * Do not transfer calibration between different cameras or resolution settings.
 *
 * Reference: https://ftc-docs.firstinspires.org/en/latest/programming_resources/vision/camera_calibration/camera-calibration.html
 */
public class CameraCalibrate {

    // Focal lengths in pixels
    public static final double FX = 249.376;
    public static final double FY = 249.376;

    // Optical center (principal point)
    public static final double CX = 1062.11;
    public static final double CY = 460.204;

    // Radial distortion coefficients (Brown's Model)
    public static final double K1 = 0.000152435;
    public static final double K2 = -8.40784e-05;
    public static final double K3 = 1.8523e-06;

    // Tangential distortion coefficients
    public static final double P1 = 0.000314673;
    public static final double P2 = 0.000336862;

    // Skew coefficient
    public static final double SKEW = 0.0;

    // Calibration quality metric
    public static final double MEAN_REPROJECTION_ERROR = 0.378256;
}
