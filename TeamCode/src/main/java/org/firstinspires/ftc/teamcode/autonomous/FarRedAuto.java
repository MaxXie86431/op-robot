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

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Configurable
@Autonomous(name = "Far 9-Ball Red Auto")
public class FarRedAuto extends NextFTCOpMode {


    public static Pose startPose = new Pose(88, 13, Math.toRadians(180));
    public static Pose topRowStartPose = new Pose(99, 84.35, Math.toRadians(180));
    public static Pose topRowEndPose = new Pose(124, 84.35, Math.toRadians(180));
    public static Pose middleRowStartPose = new Pose(99, 60, Math.toRadians(180));
    public static Pose middleRowEndPose = new Pose(124, 60, Math.toRadians(180));
    public static Pose bottomRowStartPose = new Pose(99, 35, Math.toRadians(180));
    public static Pose bottomRowEndPose = new Pose(124, 35, Math.toRadians(180));
    public static Pose frontLaunchPose = new Pose(59, 84.3, Math.toRadians(180));
    public static Pose endPose = new Pose(85, 34, Math.toRadians(180));

    public static int FAR_SPEED = 1520;
    public static int FIRST_SPEED = 1520;
    public static int SECOND_SPEED = 1520;
    public static int THIRD_SPEED = 1200;

    public static double offset = 20;
    private PathChain initialToBottomStart;
    private PathChain middleRowPath;

    private PathChain bottomRowPath;
    private PathChain topRowPath;
    private PathChain outtaTheWay;


    public static double wait = 2;

    public static Pose autoPose;
    static PoseHistory poseHistory;
    private Telemetry debugTelemetry;



    {
        addComponents(
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(Intake.INSTANCE, Flicker.INSTANCE, Flywheel.INSTANCE, Limelight.INSTANCE),
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
                new FollowPath(initialToBottomStart),
                new FollowPath(bottomRowPath),
                Flywheel.INSTANCE.out(FIRST_SPEED),
                Flicker.INSTANCE.flickTwo(1),
                new FollowPath(middleRowPath),
                Flywheel.INSTANCE.constantShot(SECOND_SPEED),
                Flicker.INSTANCE.flickTwo(1),
                new FollowPath(topRowPath),
                Flywheel.INSTANCE.out(THIRD_SPEED),
                Flicker.INSTANCE.flickTwo(1),
                Flywheel.INSTANCE.shutdown(),

                new FollowPath(outtaTheWay)
        );
    }

    public void buildPaths() {
        initialToBottomStart = follower().pathBuilder()
                .addPath(new BezierLine(startPose, bottomRowStartPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), bottomRowStartPose.getHeading())
                .build();
        topRowPath = follower().pathBuilder()
                .addPath(new BezierLine(startPose, topRowStartPose))
                .addParametricCallback(Constants.complete, () -> {
                    debugTelemetry.addData("CALLBACK", "topRowPath inward triggered");
                    debugTelemetry.update();
                    Intake.INSTANCE.in().schedule();
                })
                .addPath(new BezierLine(topRowStartPose, topRowEndPose))
                .addPath(new BezierLine(topRowEndPose, frontLaunchPose))
                .addParametricCallback(0.2, () -> {
                    debugTelemetry.addData("CALLBACK", "topRowPath stop triggered");
                    debugTelemetry.update();
                    Intake.INSTANCE.stop().schedule();
                })
                .setLinearHeadingInterpolation(topRowEndPose.getHeading(), frontLaunchPose.getHeading())
                .build();
        middleRowPath = follower().pathBuilder()
                .addPath(new BezierLine(startPose, middleRowStartPose))
                .addParametricCallback(Constants.complete, () -> {
                    debugTelemetry.addData("CALLBACK", "middleRowPath inward triggered");
                    debugTelemetry.update();
                    Intake.INSTANCE.in().schedule();
                })
                .addPath(new BezierLine(middleRowStartPose, middleRowEndPose))
                .addPath(new BezierLine(middleRowEndPose, startPose))
                .addParametricCallback(0.2, () -> {
                    debugTelemetry.addData("CALLBACK", "middleRowPath stop triggered");
                    debugTelemetry.update();
                    Intake.INSTANCE.stop().schedule();
                })
                .setLinearHeadingInterpolation(middleRowEndPose.getHeading(), startPose.getHeading())
                .build();
        bottomRowPath = follower().pathBuilder()
                .addPath(new BezierLine(bottomRowStartPose, bottomRowEndPose))
                .addParametricCallback(0, () -> {
                    debugTelemetry.addData("CALLBACK", "bottomRowPath inward triggered");
                    debugTelemetry.update();
                    Intake.INSTANCE.in().schedule();
                })
                .addPath(new BezierLine(bottomRowEndPose, startPose))
                .addParametricCallback(0.2, () -> {
                    debugTelemetry.addData("CALLBACK", "bottomRowPath stop triggered");
                    debugTelemetry.update();
                    Intake.INSTANCE.stop().schedule();
                })
                .setLinearHeadingInterpolation(bottomRowEndPose.getHeading(), startPose.getHeading())
                .build();
        outtaTheWay = follower().pathBuilder()
                .addPath(new BezierLine(frontLaunchPose, middleRowStartPose))
                .setLinearHeadingInterpolation(frontLaunchPose.getHeading(), middleRowStartPose.getHeading())
                .build();
    }

    @Override
    public void onInit() {
        Flywheel.powerState = false;
        debugTelemetry = telemetry;
        // Initialize the follower with your constants
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
        PoseStorage.setPose(follower().getPose());
        telemetry.addData("flywheel rpm: ", Flywheel.INSTANCE.getVelocityRPM());
        telemetry.update();
        follower().update();
    }

}