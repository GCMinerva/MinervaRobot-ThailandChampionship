package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotHardwareConfig;

public class Constants {
    // Drive constants using centralized config
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(RobotHardwareConfig.DriveParams.MAX_POWER)
            .rightFrontMotorName(RobotHardwareConfig.DriveMotors.FRONT_RIGHT)
            .rightRearMotorName(RobotHardwareConfig.DriveMotors.BACK_RIGHT)
            .leftRearMotorName(RobotHardwareConfig.DriveMotors.BACK_LEFT)
            .leftFrontMotorName(RobotHardwareConfig.DriveMotors.FRONT_LEFT)
            .leftFrontMotorDirection(RobotHardwareConfig.DriveDirections.FRONT_LEFT)
            .leftRearMotorDirection(RobotHardwareConfig.DriveDirections.BACK_LEFT)
            .rightFrontMotorDirection(RobotHardwareConfig.DriveDirections.FRONT_RIGHT)
            .rightRearMotorDirection(RobotHardwareConfig.DriveDirections.BACK_RIGHT)
            .xVelocity(RobotHardwareConfig.DriveParams.X_VELOCITY)
            .yVelocity(RobotHardwareConfig.DriveParams.Y_VELOCITY);
    // Localizer constants using centralized config
    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(RobotHardwareConfig.Odometry.FORWARD_POD_Y_OFFSET)
            .strafePodX(RobotHardwareConfig.Odometry.STRAFE_POD_X_OFFSET)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName(RobotHardwareConfig.Odometry.PINPOINT_NAME)
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    // Follower constants using centralized config
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(RobotHardwareConfig.RobotPhysics.MASS)
            .forwardZeroPowerAcceleration(RobotHardwareConfig.RobotPhysics.FORWARD_ZERO_POWER_ACCEL)
            .lateralZeroPowerAcceleration(RobotHardwareConfig.RobotPhysics.LATERAL_ZERO_POWER_ACCEL)
            .translationalPIDFCoefficients(new PIDFCoefficients(
                    RobotHardwareConfig.PIDFCoefficients.TRANS_P,
                    RobotHardwareConfig.PIDFCoefficients.TRANS_I,
                    RobotHardwareConfig.PIDFCoefficients.TRANS_D,
                    RobotHardwareConfig.PIDFCoefficients.TRANS_F
            ))
            .headingPIDFCoefficients(new PIDFCoefficients(
                    RobotHardwareConfig.PIDFCoefficients.HEADING_P,
                    RobotHardwareConfig.PIDFCoefficients.HEADING_I,
                    RobotHardwareConfig.PIDFCoefficients.HEADING_D,
                    RobotHardwareConfig.PIDFCoefficients.HEADING_F
            ))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(
                    RobotHardwareConfig.PIDFCoefficients.DRIVE_P,
                    RobotHardwareConfig.PIDFCoefficients.DRIVE_I,
                    RobotHardwareConfig.PIDFCoefficients.DRIVE_D,
                    RobotHardwareConfig.PIDFCoefficients.DRIVE_F,
                    RobotHardwareConfig.PIDFCoefficients.DRIVE_FILTER_DECAY
            ))
            .centripetalScaling(RobotHardwareConfig.PIDFCoefficients.CENTRIPETAL_SCALING);

    // Path constraints using centralized config
    public static PathConstraints pathConstraints = new PathConstraints(
            RobotHardwareConfig.PathConstraints.VELOCITY,
            RobotHardwareConfig.PathConstraints.ACCELERATION,
            RobotHardwareConfig.PathConstraints.ANGULAR_VELOCITY,
            RobotHardwareConfig.PathConstraints.ANGULAR_ACCELERATION
    );

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}