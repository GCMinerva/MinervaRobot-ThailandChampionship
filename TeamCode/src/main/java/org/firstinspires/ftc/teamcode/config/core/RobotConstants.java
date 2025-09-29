package org.firstinspires.ftc.teamcode.config.core;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.config.core.util.Alliance;

@Config
public class RobotConstants {

    // Intake Positions
//    public static double intakeRotateLeftOffset = -0.02;
//    public static double intakeRotateForward = 0.51+0.06;
//    public static double intakeRotateHover = 0.26+0.06; // !! 0.30; // 0.68 pivotforward
//    public static double intakeRotateGround = 0.31+0.06; // !! 0.34;
//    public static double intakeRotateHoverOutward = 0.77+0.06; // !! 0.7;
//    public static double intakeRotateHoverOutwardFull = 0.8+0.06;
//    public static double intakeRotateGroundOutward = 0.7+0.06; // !! 0.67;
//    public static double intakeRotateGroundOutwardFull = 0.72+0.06;
//    public static double intakeRotateTransfer = 0.37-0.01+0.06; // !! 0.58;
//    public static double intakeRotateThrowback = 0.6+0.06; // !! 0.62; // 0.64; // 0.48
//    public static double intakeRotateHoldback = 0.5+0.06;
//    public static double intakeRotateDodge = 0.86+0.06; // !! 0.76;
    public static double intakeRotateLeftOffset = -0.01;
    public static double intakeRotateForward = 0.51+0.06-0.06;
    public static double intakeRotateHover = 0.26+0.06-0.06; // !! 0.30 // 0.68 pivotforward
    public static double intakeRotateGround = 0.31+0.06-0.06; // !! 0.34
    public static double intakeRotateHoverOutward = 0.77+0.06+0.01-0.06; // !! 0.7
    public static double intakeRotateHoverOutwardFull = 0.8+0.06-0.06;
    public static double intakeRotateGroundOutward = 0.7+0.06+0.03-0.01-0.06-0.01; // !! 0.67
    public static double intakeRotateGroundOutwardFull = 0.72+0.06+0.03-0.02-0.06;
    public static double intakeRotateTransfer = 0.37-0.01+0.06+0.02-0.06; // !! 0.58
    public static double intakeRotateTransferWait = intakeRotateTransfer-0.05; // !! 0.30; // 0.27;
    public static double intakeRotateThrowback = 0.6+0.06-0.06; // !! 0.62 // 0.64 // 0.48
    public static double intakeRotateHoldback = 0.5+0.06-0.06;
    public static double intakeRotateDodge = 0.86+0.06-0.06; // !! 0.76
    public static double intakeRotatePerDegree = 0.08 * 4 / 360.0; // 0.27 / 360.0
    public static double intakePivotForward = 0.78; // !! 0.88;
    public static double intakePivotHover = 0.28; // !! 0.23;
    public static double intakePivotGround = 0.23; // !! 0.17;
    public static double intakePivotHoverOutward = 0.75; // !! 0.83;
    public static double intakePivotHoverOutwardFull = 0.72;
    public static double intakePivotGroundOutward = 0.82; // !! 0.90;
    public static double intakePivotGroundOutwardFull = 0.8;
    public static double intakePivotBackward = 0.24; // !! 0.18; // unused
    public static double intakePivotTransfer = 0.55; // !! 0.30; // 0.27;\
    public static double intakePivotThrowback = 0.29; // !! 0.21; // 0.18; // 0.33
    public static double intakePivotHoldback = 0.355; // !! 0.33; // 0.33

    public static double intakeGrabClosed = 0.40;
    public static double intakeGrabOpen = 0.52; // 0.76

    public static double intakeGrabOpenFull = 0.63;
    public static double intakeGrabLoose = 0.4275;

    // Outtake Positions
    public static double outtakeRotateTransfer = 0.25+0.31+0.04-0.04+0.02; // 0.34-0.09+0.03; // 0.41;
    public static double outtakeRotateScore = 0.34-0.09+0.31;
    public static double outtakeRotatePickup = 0.30-0.09+0.02+0.31; // 0.28; // ! 0.05; // 0.01 1100 straight
    public static double outtakeRotateSpecimen = 0.32-0.09+0.31-0.02+0.01; // 0.32 // ! 0.13;

    public static double outtakePivotLeftOffset = 0;
    public static double outtakePivotTransfer = 0.76-0.03-0.01+0.04; // ?? 0.76; // 0.77; // ! 0.72; // 0.75
    public static double outtakePivotScore = 0; // !! 0.08;
    public static double outtakePivotPickup = 0.98; // ?? 0.92; // !! 0.84; // ! 0.81;
    public static double outtakePivotSpecimen = 0.47+0.02; // !! 0.45;

    public static double outtakeGrabOpen = 0.53-0.08+0.02+0.02-0.05;

    public static double outtakeGrabOpenFull = 0.57-0.02+0.02+0.02-0.05;
    public static double outtakeGrabClosed = 0.42-0.03-0.02+0.02+0.02-0.05+0.01;
    public static double outtakeGrabLoose = 0.45-0.03+0.02+0.02-0.05;

    // Lift Positions
    public static int liftToZero = 10;
    public static int liftToTransfer = 1000;
    public static int liftToPickup = 1050; // ! 830;
    public static int liftToSpecimen = 1500;
    public static int liftToHighBasket = 2600;
    public static int liftToLowBasket = 1000;
    public static int liftToFinish = 1950;
    public static int liftToHang = 2600;
    public static int liftToIdle = liftToTransfer;
    public static int liftToInit = liftToZero;

    // Extend Positions
    public static double extendZero = 0.95; // 0.96 full
    public static double extendFull = 0.73; // 0.62 full
    public static double extendAutoFull = 0.68; // 0.62 full
    public static double extendGrab = 0.80; // 0.62 full
    public static double extendHoldback = 0.86; // 0.62 full
    public static double extendFirst = 0.87;
    public static double extendSecond = 0.8;

//    public static double[] extendInches = {-1,-1,0.94,0.909,0.894,0.882,0.878,0.872,0.862,0.852,
//                                            0.844,0.834,0.826,0.815,0.805,0.796,0.786,0.777,0.768,0.756,
//                                            0.748};
    public static double[] extendInches = {-1,-1,0.94,0.908,0.894,0.886,0.88,0.869,0.858,0.85,
                                            0.842,0.834,0.823,0.813,0.804,0.795,0.785,0.776,0.766,0.756,
                                            0.746};

    // Vision
    public static double visionInch = 20;
    public static int visionCameraWidth = 640;
    public static int visionCameraHeight = 360;
//    // HOUSE
//    public static int visionExposure = 50;
//    public static int visionWhiteBalance = 6500;
    // BOOTH
//    public static int visionExposure = 50;
//    public static int visionWhiteBalance = 3000;

    // FIELD
    public static int visionExposure = 30;
    public static int visionWhiteBalance = 5000;

    public static int visionGain = 0;

    // Alliance
    public static Alliance currentAlliance = Alliance.RED;
}