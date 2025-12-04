package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(11.8)
            .forwardZeroPowerAcceleration(-36.47575519127562)
            .lateralZeroPowerAcceleration(-55.47828826172785)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.08,0,0.0067,0.03))
            .headingPIDFCoefficients(new PIDFCoefficients(0.8,0,0.03,0.028));//change today



    public static PathConstraints pathConstraints = new PathConstraints(0.99,
            100,
            1,
            1);

    // ✅ Drivetrain constants (aligned with TeleOp motor names/directions)
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .leftFrontMotorName("frontLeft")
            .leftRearMotorName("backLeft")
            .rightFrontMotorName("frontRight")
            .rightRearMotorName("backRight")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity( 76.68937863327388)
            .yVelocity(55.81917974517101);

    // ✅ Pinpoint localizer constants
    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-3.2)   // adjust based on your robot’s actual offset
            .strafePodX(-5)   // adjust based on your robot’s actual offset
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")   // must match the name in your config
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)     // ✅ attach drivetrain
                .pinpointLocalizer(localizerConstants) // ✅ attach localizer
                .build();
    }
}
