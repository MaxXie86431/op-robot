package org.firstinspires.ftc.teamcode.autonomous;


import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.PoseHistory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.extensions.pedro.FollowPath;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.PoseStorage;
import org.firstinspires.ftc.teamcode.robot.Flicker;
import org.firstinspires.ftc.teamcode.robot.Flywheel;
import org.firstinspires.ftc.teamcode.robot.Intake;
import org.firstinspires.ftc.teamcode.robot.Limelight;

import dev.nextftc.ftc.components.BulkReadComponent;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Turret;

@Configurable
@Autonomous(name = "Close 3 Row Blue Auto")
public class CloseBlueAuto extends NextFTCOpMode {
    // Define poses
    private static final Pose startPose = new Pose(25, 125, Math.toRadians(140));
    private static final Pose launchPose = new Pose(54, 85, Math.toRadians(136));
    private static final Pose outtatheWayPose = new Pose(54,65,Math.toRadians(90));
    private static final Pose topRowEndPose = new Pose(23, 84, Math.toRadians(180));
    private static final Pose middleRowStartPose = new Pose(54, 58, Math.toRadians(180));
    private static final Pose middleRowEndPose = new Pose(17, 58, Math.toRadians(180));
    public static Pose leverBack = new Pose(29,58,Math.toRadians(180));
    public static Pose leverPose = new Pose(23, 65, Math.toRadians(90));
    private static final Pose bottomRowStartPose = new Pose(54, 37, Math.toRadians(180));
    private static final Pose bottomRowEndPose = new Pose(17, 37, Math.toRadians(180));

    public static double wait = 2;
    private PathChain initialLaunchPath, outtaTheWayPath, topRowPath, middleRowPath, bottomRowPath, hitLeverPath;
    public static int CLOSE_SPEED = 1150;
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
                        new FollowPath(initialLaunchPath),
                        Flywheel.INSTANCE.out(CLOSE_SPEED)
                ),
                Flicker.INSTANCE.flickThreeBallsAuto(),
                new FollowPath(middleRowPath),
                new FollowPath(hitLeverPath),
                Flicker.INSTANCE.flickThreeBallsAuto(),
                new FollowPath(topRowPath),
                Flicker.INSTANCE.flickThreeBallsAuto(),
                new FollowPath(bottomRowPath),
                Flicker.INSTANCE.flickThreeBallsAuto(),

                Flywheel.INSTANCE.shutdown(),
                new FollowPath(outtaTheWayPath)


        );
    }

    public void buildPaths() {
        outtaTheWayPath = follower().pathBuilder()
                .addPath(new BezierLine(launchPose,outtatheWayPose))
                .setLinearHeadingInterpolation(launchPose.getHeading(), outtatheWayPose.getHeading())
                .build();
        initialLaunchPath = follower().pathBuilder()
                .addPath(new BezierLine(startPose, launchPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), launchPose.getHeading())
                .build();
        topRowPath = follower().pathBuilder()
                .addPath(new BezierLine(launchPose, topRowEndPose))
                .addParametricCallback(Constants.start, () -> {
                    debugTelemetry.addData("CALLBACK", "topRowPath stop triggered");
                    debugTelemetry.update();
                    Intake.INSTANCE.in().schedule();
                })
                .addPath(new BezierLine(topRowEndPose, launchPose))
                .setLinearHeadingInterpolation(topRowEndPose.getHeading(), launchPose.getHeading())
                .addParametricCallback(Constants.start, () -> {
                    debugTelemetry.addData("CALLBACK", "middleRowPath stop triggered");
                    debugTelemetry.update();
                    Intake.INSTANCE.stop().schedule();
                    Flicker.INSTANCE.allDown();
                })
                .build();
        middleRowPath = follower().pathBuilder()
                .addPath(new BezierLine(launchPose, middleRowStartPose))
                .setLinearHeadingInterpolation(launchPose.getHeading(),middleRowStartPose.getHeading())
                .addParametricCallback(Constants.complete, () -> {
                    debugTelemetry.addData("CALLBACK", "middleRowPath inward triggered");
                    debugTelemetry.update();
                    Intake.INSTANCE.in().schedule();
                })
                .addPath(new BezierLine(middleRowStartPose, middleRowEndPose))
                .build();
        bottomRowPath = follower().pathBuilder()
                .addPath(new BezierLine(launchPose, bottomRowStartPose))
                .setLinearHeadingInterpolation(launchPose.getHeading(), bottomRowStartPose.getHeading())
                .addParametricCallback(Constants.complete, () -> {
                    debugTelemetry.addData("CALLBACK", "bottomRowPath inward triggered");
                    debugTelemetry.update();
                    Intake.INSTANCE.in().schedule();

                })
                .addPath(new BezierLine(bottomRowStartPose, bottomRowEndPose))
                .addPath(new BezierLine(bottomRowEndPose, launchPose))
                .addParametricCallback(Constants.start, () -> {
                    debugTelemetry.addData("CALLBACK", "bottomRowPath stop triggered");
                    debugTelemetry.update();
                    Intake.INSTANCE.stop().schedule();
                    Flicker.INSTANCE.allDown();
                })
                .setLinearHeadingInterpolation(bottomRowEndPose.getHeading(), launchPose.getHeading())
                .build();
        hitLeverPath = follower().pathBuilder()
                .addPath(new BezierLine(middleRowEndPose, leverBack))
                .addParametricCallback(Constants.start, () -> {
                    debugTelemetry.addData("CALLBACK", "middleRowPath stop triggered");
                    debugTelemetry.update();
                    Intake.INSTANCE.stop().schedule();
                    Flicker.INSTANCE.allDown();
                })
                .setConstantHeadingInterpolation(180)
                .addPath(new BezierLine(leverBack, leverPose))
                .setConstantHeadingInterpolation(leverPose.getHeading())
                .addParametricCallback(Constants.complete, () -> {
                    new Delay(1);
                })
                .addPath(new BezierLine(leverPose, launchPose))
                .setLinearHeadingInterpolation(leverPose.getHeading(), launchPose.getHeading())
                .build();

    }

    @Override
    public void onInit() {
        Flywheel.powerState = false;
        Turret.powerState =false;
        Turret.INSTANCE.zero();
        Flicker.INSTANCE.setFlickDelay(Flicker.flickDelayAuto);
        debugTelemetry = telemetry;
        Flicker.INSTANCE.flickThreeBallsAuto().schedule();
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