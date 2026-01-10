package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
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

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(9.52)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.12, 0, 0.008, 0.15))
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.8,0,0.9,0.15))
            .headingPIDFCoefficients(new PIDFCoefficients(1,0,0.02,0.15))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(7,0,8,0))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.14,0,0.009,0.6,0.15))
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.002, 0,0.0005,0.6,0.1))
            .centripetalScaling(0.0005)
            .lateralZeroPowerAcceleration(-58.2835)
            .forwardZeroPowerAcceleration(-43.30156);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("FRM")
            .rightRearMotorName("BRM")
            .leftFrontMotorName("FLM")
            .leftRearMotorName("BLM")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD )
            .xVelocity(51.704)
            .yVelocity(38.2032)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-5.5)
            .strafePodX(7.375)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .pinpointLocalizer(localizerConstants)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}
