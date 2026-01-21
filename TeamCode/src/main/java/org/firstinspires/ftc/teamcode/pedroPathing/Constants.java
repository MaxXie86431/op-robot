package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class Constants {
    public static double start = 0.2;
    public static double complete = 0.9;

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(13.97)
            .forwardZeroPowerAcceleration(-41.61821880156717)
            .lateralZeroPowerAcceleration(-68.90824821816265)
            .translationalPIDFCoefficients(new PIDFCoefficients(
                    0.12,
                    0,
                    0.01,
                    0.03
            ))
            .headingPIDFCoefficients(new PIDFCoefficients(
                    0.9,
                    0,
                    0.01,
                    0.03
            ))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(
                    0.010,
                    0,
                    0.000002,
                    0.6,
                    0
            ))
            .centripetalScaling(0.0001);



    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .leftFrontMotorName("Top-Left-Motor")
            .leftRearMotorName("Bottom-Left-Motor")
            .rightFrontMotorName("Top-Right-Motor")
            .rightRearMotorName("Bottom-Right-Motor")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(61.183044)
            .yVelocity(47.247518224040356)
            .useBrakeModeInTeleOp(true);


    public static PinpointConstants localizerConstants = new PinpointConstants()
            .distanceUnit(DistanceUnit.INCH)
            .forwardPodY(-3.125)
            .strafePodX(-1.75)
            //.hardwareMapName("pinpoint")
            .encoderResolution(
                    GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD
            )
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static PathConstraints pathConstraints = new PathConstraints(
            0.995,
            0.1,
            0.1,
            0.009,
            500,
            0.85,
            10,
            1
    );

    //public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .build();
    }
}