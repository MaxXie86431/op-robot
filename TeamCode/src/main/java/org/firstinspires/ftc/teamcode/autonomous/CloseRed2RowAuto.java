package org.firstinspires.ftc.teamcode.autonomous;


import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.drawCurrent;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.drawCurrentAndHistory;

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
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.extensions.pedro.FollowPath;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.PoseStorage;
import org.firstinspires.ftc.teamcode.pedroPathing.Tuning;
import org.firstinspires.ftc.teamcode.robot.Flicker;
import org.firstinspires.ftc.teamcode.robot.Flywheel;
import org.firstinspires.ftc.teamcode.robot.Intake;
import org.firstinspires.ftc.teamcode.robot.Limelight;

import dev.nextftc.ftc.components.BulkReadComponent;
import kotlin.time.Instant;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Turret;

@Configurable
@Autonomous(name = "Close 2 Row Red Auto")
public class CloseRed2RowAuto extends NextFTCOpMode {
    // Define poses
    private static final Pose startPose = new Pose(119, 125, Math.toRadians(45));
    private static final Pose launchPose = new Pose(84, 84.3, Math.toRadians(45));
    private static final Pose outtatheWayPose = new Pose(94,65,Math.toRadians(300));
    private static final Pose parkPose = new Pose(38.5,34,225);
    private static final Pose topRowEndPose = new Pose(120, 84.35, Math.toRadians(0));
    private static final Pose middleRowStartPose = new Pose(84, 60, Math.toRadians(0));
    private static final Pose middleRowEndPose = new Pose(120, 60, Math.toRadians(0));
    private static final Pose bottomRowStartPose = new Pose(84, 36, Math.toRadians(0));
    private static final Pose bottomRowEndPose = new Pose(120, 36, Math.toRadians(0));

    public static double wait = 2;
    private PathChain initialLaunchPath, initialOut, outtaTheWayPath, topRowPath, middleRowPath, bottomRowPath, parkPath;
    public static int CLOSE_SPEED = 1200;
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
                Flicker.INSTANCE.flickTwo(1),
                new FollowPath(topRowPath),
                Flicker.INSTANCE.flickTwo(1),
                new FollowPath(middleRowPath),
                Flicker.INSTANCE.flickTwo(1),

                Flywheel.INSTANCE.shutdown(),
                new FollowPath(outtaTheWayPath)


        );
    }

    public void buildPaths() {
        initialOut = follower().pathBuilder()
                .addPath(new BezierLine(startPose,outtatheWayPose))
                .setTangentHeadingInterpolation()
                .build();
        outtaTheWayPath = follower().pathBuilder()
                .addPath(new BezierLine(launchPose,outtatheWayPose))
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
                .addParametricCallback(Constants.start, () -> {
                    debugTelemetry.addData("CALLBACK", "middleRowPath stop triggered");
                    debugTelemetry.update();
                    Intake.INSTANCE.stop().schedule();
                })
                .setLinearHeadingInterpolation(middleRowEndPose.getHeading(), launchPose.getHeading())
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
                })
                .setLinearHeadingInterpolation(bottomRowEndPose.getHeading(), launchPose.getHeading())
                .build();
        parkPath = follower().pathBuilder()
                .addPath(new BezierLine(launchPose,parkPose))
                .build();
    }

    @Override
    public void onInit() {
        Flywheel.powerState = false;
        debugTelemetry = telemetry;
        // Initialize the follower with your constants
        Flicker.INSTANCE.allDown();
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