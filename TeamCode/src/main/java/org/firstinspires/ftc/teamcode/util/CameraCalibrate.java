package org.firstinspires.ftc.teamcode.util;

/**
 * Camera Calibration Data
 * Generated using 3DF Zephyr or MRCAL calibration tool
 *
 * Calibration Pattern Results:
 *
 * Focals (pixels):
 *   Fx: 1449.18
 *   Fy: 1449.18
 *
 * Optical Center:
 *   Cx: 954.863
 *   Cy: 544.831
 *
 * Radial Distortion (Brown's Model):
 *   K1: 0.0421727
 *   K2: -0.205409
 *   K3: 0.184011
 *   P1: -0.00186866
 *   P2: -0.00161809
 *
 * Skew: 0
 *
 * Statistics:
 *   Mean Square Reprojection Error: 0.370918 pixels
 *
 * Note: This calibration is specific to the camera, lens, and resolution used.
 * Do not transfer calibration between different cameras or resolution settings.
 *
 * Reference: https://ftc-docs.firstinspires.org/en/latest/programming_resources/vision/camera_calibration/camera-calibration.html
 */
public class CameraCalibrate {

    // Focal lengths in pixels
    public static final double FX = 1449.18;
    public static final double FY = 1449.18;

    // Optical center (principal point)
    public static final double CX = 954.863;
    public static final double CY = 544.831;

    // Radial distortion coefficients (Brown's Model)
    public static final double K1 = 0.0421727;
    public static final double K2 = -0.205409;
    public static final double K3 = 0.184011;

    // Tangential distortion coefficients
    public static final double P1 = -0.00186866;
    public static final double P2 = -0.00161809;

    // Skew coefficient
    public static final double SKEW = 0.0;

    // Calibration quality metric
    public static final double MEAN_REPROJECTION_ERROR = 0.370918;
}
