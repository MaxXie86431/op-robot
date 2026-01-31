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
@Autonomous(name = "Far Human Red Auto")
public class FarRedHumanAuto extends NextFTCOpMode {
    private static final Pose startPose = new Pose(88, 13, Math.toRadians(72));
    private static final Pose humanPlayerPose = new Pose(133, 10, Math.toRadians(0));
    public static int FAR_SPEED = 1520;
    public static double initialDelay = 10;
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
                new ParallelGroup(
                        //new FollowPath(initialLaunchPath),
                        Flywheel.INSTANCE.out(FAR_SPEED)
                ),
                Flicker.INSTANCE.flickTwo(1),

                new Delay(initialDelay),
                new FollowPath(initialToHumanPlayer),
                Flicker.INSTANCE.flickTwo(1),
                new Delay(delayBetweenCycles),
                new FollowPath(initialToHumanPlayer),
                Flicker.INSTANCE.flickTwo(1),
                new Delay(delayBetweenCycles),
                new FollowPath(initialToHumanPlayer),
                Flicker.INSTANCE.flickTwo(1),
                new Delay(delayBetweenCycles),
                new FollowPath(initialToHumanPlayer),
                Flicker.INSTANCE.flickTwo(1),
                new Delay(delayBetweenCycles),
                new FollowPath(initialToHumanPlayer),
                Flicker.INSTANCE.flickTwo(1),
                new Delay(delayBetweenCycles),
                new FollowPath(initialToHumanPlayer),
                Flicker.INSTANCE.flickTwo(1),
                new Delay(delayBetweenCycles),

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
                .addPath(new BezierLine(humanPlayerPose, startPose))
                .addParametricCallback(0.2, () -> {
                    debugTelemetry.addData("CALLBACK", "humanPlayerPath stop triggered");
                    debugTelemetry.update();
                    Intake.INSTANCE.stop().schedule();
                })
                .setLinearHeadingInterpolation(humanPlayerPose.getHeading(), startPose.getHeading())
                .build();
        outtaTheWay = follower().pathBuilder()
                .addPath(new BezierLine(startPose, humanPlayerPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), humanPlayerPose.getHeading())
                .build();
    }

    @Override
    public void onInit() {
        Flywheel.powerState = false;
        debugTelemetry = telemetry;
        // Initialize the follower with your constants
        Flicker.INSTANCE.allDown().schedule();
        Turret.INSTANCE.zero();
        PoseStorage.setPose(startPose);
        follower().setStartingPose(startPose);
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