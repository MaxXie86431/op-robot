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
    private static final Pose startPose = new Pose(54, 12, Math.toRadians(111));
    private static final Pose launchPose = new Pose(54, 12, Math.toRadians(111));
    private static final Pose humanPlayerPose = new Pose(12, 13, Math.toRadians(90));
    private static final Pose moveBackPose = new Pose(40, 13,Math.toRadians(90));
    public static int FAR_SPEED = 1500;
    public static double initialDelay = 10;
    public static double delayBetweenCycles = 2;
    private PathChain launchToHumanPlayer;
    private PathChain initialtoLaunchPose;
    private PathChain outtaTheWay;
    static PoseHistory poseHistory;
    private Telemetry debugTelemetry;
    public static double initialAngle = -71;



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
                new ParallelGroup(
                        //new FollowPath(initialLaunchPath),
                        Flywheel.INSTANCE.out(FAR_SPEED)
                ),
                Flicker.INSTANCE.flickThreeBallsAuto(),
                Intake.INSTANCE.in(),
                new FollowPath(launchToHumanPlayer),
                Flicker.INSTANCE.flickThreeBallsAuto(),
                //new Delay(delayBetweenCycles),
                new FollowPath(launchToHumanPlayer),
                Flicker.INSTANCE.flickThreeBallsAuto(),
                Flywheel.INSTANCE.shutdown(),
                new FollowPath(outtaTheWay)
        );
    }

    public void buildPaths() {
        launchToHumanPlayer = follower().pathBuilder()
                .addPath(new BezierLine(launchPose, humanPlayerPose))
                .addPath(new BezierLine(humanPlayerPose, moveBackPose))
                .setConstantHeadingInterpolation(humanPlayerPose.getHeading())
                .addPath(new BezierLine(moveBackPose, humanPlayerPose))
                .addPath(new BezierLine(humanPlayerPose, launchPose))
                .setLinearHeadingInterpolation(humanPlayerPose.getHeading(), launchPose.getHeading())
                .build();
        initialtoLaunchPose = follower().pathBuilder()
                .addPath(new BezierLine(startPose, launchPose))
                .setLinearHeadingInterpolation(humanPlayerPose.getHeading(), launchPose.getHeading())
                .build();
        outtaTheWay = follower().pathBuilder()
                .addPath(new BezierLine(launchPose, humanPlayerPose))
                .setLinearHeadingInterpolation(launchPose.getHeading(), humanPlayerPose.getHeading())
                .build();
    }

    @Override
    public void onInit() {
        Flywheel.powerState = false;
        Turret.powerState = false;
        Turret.INSTANCE.zero();
        Flicker.betweenflicksDelayAuto = 0.4;
        Flicker.INSTANCE.setFlickDelay(0.6);
        debugTelemetry = telemetry;
        // Initialize the follower with your constants
        PoseStorage.setPose(launchPose);
        follower().setStartingPose(launchPose);
        follower().update();
        buildPaths();
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
        telemetry.update();
    }

}
