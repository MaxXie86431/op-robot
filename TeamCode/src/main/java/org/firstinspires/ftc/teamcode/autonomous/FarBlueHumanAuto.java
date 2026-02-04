package org.firstinspires.ftc.teamcode.autonomous;


import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.PoseHistory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.PoseStorage;
import org.firstinspires.ftc.teamcode.pedroPathing.Team;
import org.firstinspires.ftc.teamcode.robot.Flicker;
import org.firstinspires.ftc.teamcode.robot.Flywheel;
import org.firstinspires.ftc.teamcode.robot.Intake;
import org.firstinspires.ftc.teamcode.robot.Limelight;
import org.firstinspires.ftc.teamcode.robot.Turret;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Configurable
@Autonomous(name = "Far Human Blue Auto")
public class FarBlueHumanAuto extends NextFTCOpMode {

    public static Pose startPose = new Pose(59, 13, Math.toRadians(112));
    private static final Pose humanPlayerPose = new Pose(14, 14, Math.toRadians(180));
    //private static final Pose firstTwoPose = new Pose(13, 16, Math.toRadians(180));
    //private static final Pose lastOnePose = new Pose(13, 12, Math.toRadians(180));
    private static final Pose moveBackPose = new Pose(40, 14, Math.toRadians(180));
    public static int FAR_SPEED = 1460;
    public static double initialDelay = 2;
    public static double delayBetweenCycles = 2;
    private PathChain initialToHumanPlayer;
    private PathChain outtaTheWay;
    static PoseHistory poseHistory;
    private Telemetry debugTelemetry;



    {
        addComponents(
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(Intake.INSTANCE, Flicker.INSTANCE, Flywheel.INSTANCE, Limelight.INSTANCE, Turret.INSTANCE),
                BulkReadComponent.INSTANCE
        );
    }
    //open (0.2) is logo on left closed (0) is logo on right
    //private Command moveServo = new SetPosition(servo, 0.2).requires(this);

    private Command autonomousRoutine(){
        return new SequentialGroup(
                Turret.INSTANCE.autoTrackButton(),
                Flywheel.INSTANCE.rampedOut(FAR_SPEED),
                new Delay(initialDelay),
                Flicker.INSTANCE.flickThreeBalls(),
                new Delay(initialDelay),
                Flicker.INSTANCE.flickThreeBalls(),
                new FollowPath(initialToHumanPlayer),
                Flicker.INSTANCE.flickThreeBalls(),
                new FollowPath(initialToHumanPlayer),
                Flicker.INSTANCE.flickThreeBalls(),
                new FollowPath(initialToHumanPlayer),
                Flicker.INSTANCE.flickThreeBalls(),
                Flywheel.INSTANCE.shutdown(),
                new FollowPath(outtaTheWay)
        );
    }

    public void buildPaths() {
        initialToHumanPlayer = follower().pathBuilder()
                .addPath(new BezierLine(startPose, humanPlayerPose))
                .addParametricCallback(0, () -> {
                    debugTelemetry.addData("CALLBACK", "humanPlayerPath intake triggered");
                    debugTelemetry.update();
                    Intake.INSTANCE.in().schedule();
                })
                .addPath(new BezierLine(humanPlayerPose, moveBackPose))
                .setConstantHeadingInterpolation(humanPlayerPose.getHeading())
                .addPath(new BezierLine(moveBackPose, humanPlayerPose))
                .addPath(new BezierLine(humanPlayerPose, startPose))
                .setLinearHeadingInterpolation(humanPlayerPose.getHeading(), startPose.getHeading())
                .build();
        outtaTheWay = follower().pathBuilder()
                .addPath(new BezierLine(startPose, humanPlayerPose))
                .addParametricCallback(0.2, () -> {
                    debugTelemetry.addData("CALLBACK", "humanPlayerPath stop triggered");
                    debugTelemetry.update();
                    Intake.INSTANCE.stop().schedule();
                })
                .setLinearHeadingInterpolation(startPose.getHeading(), humanPlayerPose.getHeading())
                .build();
    }

    @Override
    public void onInit() {

        Flywheel.powerState = false;
        debugTelemetry = telemetry;
        Turret.powerState = false;
        Turret.locked = false;
        Turret.INSTANCE.setEncoderValue(0);
        Intake.INSTANCE.stop();
        //PoseStorage.resetPose();

        PoseStorage.setPose(startPose);
        follower().setStartingPose(startPose);
        follower().update();
        buildPaths();
        Team.setTeam(0);
    }


    @Override
    public void onStartButtonPressed() {
        autonomousRoutine().schedule();
    }

    @Override
    public void onUpdate() {
        follower().update();
        PoseStorage.setPose(follower().getPose());
        telemetry.addData("flywheel rpm: ", Flywheel.INSTANCE.getVelocityRPM());
        telemetry.addData("velocity required: ", FAR_SPEED);
        telemetry.addData("Limelight angle", Limelight.INSTANCE.calculateAlignmentAngle());
        telemetry.update();
    }

}