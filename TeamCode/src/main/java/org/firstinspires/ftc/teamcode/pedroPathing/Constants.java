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
    public static double start = 0.1;
    public static double complete = 0.9;

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(13.97)
            .forwardZeroPowerAcceleration(-89.28826596302258)
            .lateralZeroPowerAcceleration(-138.61248638334152)
            .translationalPIDFCoefficients(new PIDFCoefficients(
                    0.2,
                    0,
                    0.005,
                    0.03
            ))
            .headingPIDFCoefficients(new PIDFCoefficients(
                    1.5,
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
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(51.01439756859006)
            .yVelocity(43.91525628998524)
            .useBrakeModeInTeleOp(true);


    public static PinpointConstants localizerConstants = new PinpointConstants()
            .distanceUnit(DistanceUnit.INCH)
            .forwardPodY(-7.5)
            .strafePodX(6)
            //.hardwareMapName("pinpoint")
            .encoderResolution(
                    GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD
            )
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    public static PathConstraints pathConstraints = new PathConstraints(
            0.995,
            0.1,
            0.1,
            0.009,
            500,
            1,
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