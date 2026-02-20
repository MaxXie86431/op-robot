package org.firstinspires.ftc.teamcode.autonomous;


import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.PoseHistory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.extensions.pedro.FollowPath;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.PoseStorage;
import org.firstinspires.ftc.teamcode.robot.ColorDetector;
import org.firstinspires.ftc.teamcode.robot.Flicker;
import org.firstinspires.ftc.teamcode.robot.Flywheel;
import org.firstinspires.ftc.teamcode.robot.Intake;
import org.firstinspires.ftc.teamcode.robot.Limelight;

import dev.nextftc.ftc.components.BulkReadComponent;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Turret;

@Configurable
@Autonomous(name = "Close 3 Row Red Auto")
public class CloseRedAuto extends NextFTCOpMode {
    // Define poses
    public static Pose startPose = new Pose(123, 120, Math.toRadians(43));
    public static Pose launchPose = new Pose(90, 84, Math.toRadians(47));
    private static Pose topRowStartPose = new Pose(90, 80, Math.toRadians(0));
    public static Pose outtatheWayPose = new Pose(90,65,Math.toRadians(90));
    public static Pose topRowEndPose = new Pose(127, 81, Math.toRadians(0));
    public static Pose middleRowStartPose = new Pose(90, 55, Math.toRadians(0));
    public static Pose middleRowEndPose = new Pose(125, 55, Math.toRadians(0));
    public static Pose leverBack = new Pose(119,72,Math.toRadians(90));
    public static Pose leverPose = new Pose(125, 72, Math.toRadians(90));
    public static Pose bottomRowStartPose = new Pose(90, 34, Math.toRadians(0));
    public static Pose bottomRowEndPose = new Pose(127, 34, Math.toRadians(0));

    public static double wait = 2;
    public static double limit = 1;
    public static double leverDelay = 0.5;
    private PathChain initialLaunchPath, secondInitialLaunchPath, outtaTheWayPath, topRowPath, middleRowPath, bottomRowPath, hitLeverPath, leaveLeverPath;
    public static int CLOSE_SPEED = 1150;
    static PoseHistory poseHistory;
    private Telemetry debugTelemetry;



    {
        addComponents(
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(Intake.INSTANCE, Flicker.INSTANCE, Flywheel.INSTANCE, Limelight.INSTANCE, Turret.INSTANCE, ColorDetector.INSTANCE),
                BulkReadComponent.INSTANCE
        );
    }
    //open (0.2) is logo on left closed (0) is logo on right
    //private Command moveServo = new SetPosition(servo, 0.2).requires(this);

    private Command autonomousRoutine(){
        return new SequentialGroup(
                new ParallelGroup(
                        new FollowPath(initialLaunchPath, true, limit),
                        Flywheel.INSTANCE.out(CLOSE_SPEED)
                ),
                Flicker.INSTANCE.flickThreeBallsAuto(),
                new FollowPath(topRowPath),
                new FollowPath(hitLeverPath),
                new Delay(leverDelay),
                new FollowPath(leaveLeverPath),
                Flicker.INSTANCE.flickThreeBallsAuto(),
                new FollowPath(middleRowPath),
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
                .addParametricCallback(0.5, () -> {
                    new SequentialGroup(
                            new InstantCommand(() -> Flicker.motif = Limelight.INSTANCE.color())
                            //Turret.INSTANCE.turnByDegrees(-90)
                    ).schedule();

                })
                .build();

        topRowPath = follower().pathBuilder()
                .addPath(new BezierLine(launchPose,topRowStartPose))
                .setLinearHeadingInterpolation(launchPose.getHeading(),topRowStartPose.getHeading())
                .addPath(new BezierLine(topRowStartPose, topRowEndPose))
                .setLinearHeadingInterpolation(topRowStartPose.getHeading(),topRowEndPose.getHeading())
                .addParametricCallback(Constants.start, () -> {
                    debugTelemetry.addData("CALLBACK", "topRowPath stop triggered");
                    debugTelemetry.update();
                    Intake.INSTANCE.in().schedule();
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
                .addPath(new BezierLine(middleRowEndPose, launchPose))
                .setLinearHeadingInterpolation(middleRowEndPose.getHeading(), launchPose.getHeading())
                .addParametricCallback(Constants.start, () -> {
                    debugTelemetry.addData("CALLBACK", "middleRowPath stop triggered");
                    debugTelemetry.update();
                    Intake.INSTANCE.stop().schedule();
                    Flicker.INSTANCE.allDown();
                })
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
                .addPath(new BezierLine(topRowEndPose, leverBack))
                .addParametricCallback(Constants.start, () -> {
                    debugTelemetry.addData("CALLBACK", "middleRowPath stop triggered");
                    debugTelemetry.update();
                    Intake.INSTANCE.stop().schedule();
                    Flicker.INSTANCE.allDown();
                })
                .setLinearHeadingInterpolation(topRowEndPose.getHeading(),leverBack.getHeading())
                .addPath(new BezierLine(leverBack, leverPose))
                .setLinearHeadingInterpolation(leverBack.getHeading(),leverPose.getHeading())
                .build();
        leaveLeverPath = follower().pathBuilder()
                .addPath(new BezierLine(leverPose, launchPose))
                .setLinearHeadingInterpolation(leverPose.getHeading(), launchPose.getHeading())
                .build();

    }

    @Override
    public void onInit() {
        Flywheel.powerState = false;
        Turret.powerState =false;
        Turret.INSTANCE.zero();
        //Turret.INSTANCE.turnByDegrees(90).schedule();
        Flicker.INSTANCE.setFlickDelay(Flicker.flickDelayAuto);
        Flicker.betweenflicksDelayAuto = 0.1;
        debugTelemetry = telemetry;
        //Flicker.INSTANCE.flickThreeBallsAuto().schedule();

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
        telemetry.addData("motif", Flicker.motif);
        telemetry.update();
    }

}