package org.firstinspires.ftc.teamcode.utils;

import static org.firstinspires.ftc.teamcode.robot.Flywheel.launchPower;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.PoseStorage;
import org.firstinspires.ftc.teamcode.pedroPathing.Team;
import org.firstinspires.ftc.teamcode.robot.ColorDetector;
import org.firstinspires.ftc.teamcode.robot.Flywheel;
import org.firstinspires.ftc.teamcode.robot.Intake;
import org.firstinspires.ftc.teamcode.robot.Flicker;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.Limelight;
import org.firstinspires.ftc.teamcode.robot.Turret;


import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelDeadlineGroup;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.core.units.Angle;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.extensions.pedro.PedroDriverControlled;
import dev.nextftc.extensions.pedro.TurnBy;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Configurable
@TeleOp(name = "regression")
public class RegressionModel extends NextFTCOpMode {
    public static double power = 0.65;
    private double distance;
    public static double anglefactor=-1.6;
    public static double goalHeightInches = 29.5;
    public static double llDelay = 1.25;
    private double distanceToGoal = 0;

    double angle = 0;
    double angleToGoal = 0;
    private Command driverControlled = new PedroDriverControlled(
            Gamepads.gamepad1().leftStickY().negate(),
            Gamepads.gamepad1().leftStickX().negate(),
            Gamepads.gamepad1().rightStickX().negate()
    );



    private Command turns(double angle){
        return new SequentialGroup(
                new ParallelDeadlineGroup(
                        new Delay(llDelay),
                        new TurnBy(Angle.fromDeg(angle))
                ),
                driverControlled
        );
    }
    public RegressionModel() {
        addComponents(
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(Flywheel.INSTANCE, Intake.INSTANCE, Flicker.INSTANCE, Turret.INSTANCE, Limelight.INSTANCE, ColorDetector.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }


    @Override
    public void onUpdate() {
        follower().update();
        PoseStorage.setPose(follower().getPose());
        telemetry.addData("Current Pose: ", PoseStorage.getPose());
        telemetry.addData("Launch Power", launchPower);
        telemetry.addData("Target rpm", Flywheel.INSTANCE.getPower()*2000);
        telemetry.addData("Velocity RPM", Flywheel.INSTANCE.getVelocityRPM());
        telemetry.addData("D-pad Left", gamepad1.dpad_left);
        telemetry.addData("D-pad Down", gamepad1.dpad_down);
        telemetry.addData("Target X", angle);
        telemetry.addData("Target Y", angleToGoal);
        telemetry.addData("Distance", distanceToGoal);

        telemetry.update();
        super.onUpdate();
    }

    @Override
    public void onInit() {
        Team.setTeam(0);
        follower().setStartingPose(PoseStorage.getPose());
        follower().update();
        PanelsConfigurables.INSTANCE.refreshClass(this);

    }
    @Override
    public void onStartButtonPressed() {
        Flicker.INSTANCE.flickThreeBallsAuto().schedule();
        driverControlled.schedule();

        Gamepads.gamepad1().rightTrigger().greaterThan(0.2)
                .whenBecomesTrue(() -> Flywheel.INSTANCE.regress().schedule())
                .whenBecomesFalse(() -> Flywheel.INSTANCE.shutdown().schedule());

        Gamepads.gamepad1().rightBumper()
                .whenBecomesTrue(() -> Flywheel.INSTANCE.reverse().schedule())
                .whenBecomesFalse(() -> Flywheel.INSTANCE.shutdown().schedule());

        Gamepads.gamepad1().leftTrigger().greaterThan(0.2)
                .whenBecomesTrue(() -> {
                    Intake.INSTANCE.in().schedule();
                })
                .whenBecomesFalse(() -> {
                    new ParallelGroup(
                            Intake.INSTANCE.stop(),
                            Flicker.INSTANCE.flickdown1()
                    ).schedule();
                });

        Gamepads.gamepad1().leftBumper()
                .whenBecomesTrue(() -> Intake.INSTANCE.out().schedule())
                .whenBecomesFalse(() -> Intake.INSTANCE.stop().schedule());
        // △-Y, ○-B, ×-A, □-X
        Gamepads.gamepad1().x()
                .whenBecomesTrue(() -> Flicker.INSTANCE.flick1().schedule());
        Gamepads.gamepad1().y()
                .whenBecomesTrue(() -> Flicker.INSTANCE.flick2().schedule());
        Gamepads.gamepad1().b()
                .whenBecomesTrue(() -> Flicker.INSTANCE.flick3().schedule());

        Gamepads.gamepad1().dpadDown()
                .whenBecomesTrue(Flywheel.INSTANCE.decreasePower());

        Gamepads.gamepad1().dpadLeft()
                .whenBecomesTrue(() -> Flicker.INSTANCE.flickThreeBallsAuto().schedule());
        Gamepads.gamepad1().dpadRight()
                .whenBecomesTrue(() -> Flicker.INSTANCE.allDown().schedule());


        Gamepads.gamepad1().dpadUp()
                .whenBecomesTrue(Flywheel.INSTANCE.increasePower());

        Gamepads.gamepad1().a()
                .whenBecomesTrue(() -> {
                    distanceToGoal = GenetonUtils.INSTANCE.getTargetLength();
                    //distanceToGoal = Limelight.INSTANCE.calculateLaunchPower()[0]; //Limelight
                });


    }
}