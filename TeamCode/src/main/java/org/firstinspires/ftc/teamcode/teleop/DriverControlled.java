package org.firstinspires.ftc.teamcode.teleop;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.PoseStorage;
import org.firstinspires.ftc.teamcode.pedroPathing.Team;
import org.firstinspires.ftc.teamcode.robot.ColorDetector;
import org.firstinspires.ftc.teamcode.robot.Flicker;
import org.firstinspires.ftc.teamcode.robot.Flywheel;
import org.firstinspires.ftc.teamcode.robot.Intake;
import org.firstinspires.ftc.teamcode.robot.LED;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.Limelight;
import org.firstinspires.ftc.teamcode.robot.Turret;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelDeadlineGroup;
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
import dev.nextftc.hardware.driving.DriverControlledCommand;

@Configurable
@TeleOp(name = "Driver Control")
public class DriverControlled extends NextFTCOpMode {
    //left is up right is down rn
    public static double llDelay = 1.25;
    public static boolean turret = true;

    private DriverControlledCommand driverControlled = new PedroDriverControlled(
            Gamepads.gamepad1().leftStickY().negate(),
            Gamepads.gamepad1().leftStickX().negate(),
            Gamepads.gamepad1().rightStickX().negate());

    private Command turns(double angle) {
        return new SequentialGroup(
                new ParallelDeadlineGroup(
                        new Delay(llDelay),
                        new TurnBy(Angle.fromDeg(angle))),
                // Return control to driver after turn completes
                driverControlled);
    }

    public DriverControlled() {
        addComponents(
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(Flywheel.INSTANCE, Intake.INSTANCE, Flicker.INSTANCE, Turret.INSTANCE, Limelight.INSTANCE, LED.INSTANCE, ColorDetector.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE);
    }

    @Override
    public void onUpdate() {
        /*
        telemetry.addData("Velocity RPM", Flywheel.INSTANCE.getVelocityRPM());
        telemetry.addData("Distance from goal inside subsystem", Flywheel.distanceToGoal); // Flywheel.INSTANCE.distance);
        telemetry.addData("Goal Velocity inside subsystem: ", Flywheel.launchVelocity); // goalVelocity);
        telemetry.addData("Encoder Value of Turret: ", Turret.INSTANCE.getEncoderValue());
        */
        follower().update();
        PoseStorage.setPose(follower().getPose());
        telemetry.addData("Current pose", PoseStorage.getPose());
        telemetry.addData("color sensor values", ColorDetector.INSTANCE.getSensorValues());
        telemetry.addData("Goal velocity", Limelight.goalVelocity);
        telemetry.addData("LED is on", LED.on);
        telemetry.addData("Distance from goal inside subsystem", Flywheel.distanceToGoal);
        telemetry.addData("current RPM", Flywheel.INSTANCE.getVelocityRPM());
        telemetry.addData("encoder value", Turret.INSTANCE.getEncoderValue());
        telemetry.addData("team:", Team.getTeam());
        telemetry.addData("goal angle:", Turret.INSTANCE.getFieldTargetAngle());
        telemetry.addData("actual angle needed to turn: ", Turret.INSTANCE.getTargetTurretAngle());
        telemetry.update();
        super.onUpdate();
    }

    @Override
    public void onInit() {
        follower().setStartingPose(new Pose(72,72,Math.toRadians(90)));
        PoseStorage.setPose(new Pose(72,72,Math.toRadians(90)));
        Flywheel.powerState = false;
        Turret.powerState = false;
        Turret.INSTANCE.zero();
        Flicker.INSTANCE.flickThreeBalls().schedule();
        Turret.INSTANCE.zero();

    }

    @Override
    public void onStartButtonPressed() {
        /*
         * Triangle (y)
         * Square (x)
         * Cross (a)
         * Circle (b)
         */
        //driverControlled.setScalar(speed);
        driverControlled.schedule();
        //Turret.INSTANCE.autoTrack.schedule();


        Gamepads.gamepad1().rightTrigger().greaterThan(0.2)
                .whenBecomesTrue(() -> {
                    Flywheel.INSTANCE.shootOut().schedule();
                })
                .whenBecomesFalse(() -> {
                    Flywheel.INSTANCE.shutdown().schedule();
                });

        Gamepads.gamepad1().leftTrigger().greaterThan(0.2)
                .whenBecomesTrue(() -> {
                    Intake.INSTANCE.in().schedule();
                })
                .whenBecomesFalse(() -> {
                    Intake.INSTANCE.stop().schedule();
                });

        Gamepads.gamepad1().leftBumper()
                .whenBecomesTrue(() -> {
                    Intake.INSTANCE.out().schedule();
                })
                .whenBecomesFalse(() -> {
                    Intake.INSTANCE.stop().schedule();
                });



        Gamepads.gamepad1().x()
                .whenBecomesTrue(() -> {
                    Flicker.INSTANCE.flick1Switch().schedule();
                });


        Gamepads.gamepad1().y()
                .whenBecomesTrue(() -> {
                    Flicker.INSTANCE.flick2Switch().schedule();
                });

        Gamepads.gamepad1().b()
                .whenBecomesTrue(() -> {
                    Flicker.INSTANCE.flick3Switch().schedule();
                });
        Gamepads.gamepad1().a()
                .whenBecomesTrue(() -> {
                    Turret.INSTANCE.autoAlign().schedule();
                });

        Gamepads.gamepad1().dpadUp()
                .toggleOnBecomesTrue()
                .whenBecomesTrue(() -> {
                    Turret.INSTANCE.autoTrackButton().schedule();
                });


        Gamepads.gamepad1().dpadLeft()
                .whenBecomesTrue(() -> {
                    Turret.INSTANCE.turnLeft().schedule();
                })
                .whenBecomesFalse(() -> {
                    Turret.INSTANCE.stop().schedule();
                });
        Gamepads.gamepad1().dpadRight()
                .whenBecomesTrue(() -> {
                    Turret.INSTANCE.turnRight().schedule();
                })
                .whenBecomesFalse(() -> {
                    Turret.INSTANCE.stop().schedule();
                });

    }
}