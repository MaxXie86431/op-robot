package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Flicker;
import org.firstinspires.ftc.teamcode.robot.Flywheel;
import org.firstinspires.ftc.teamcode.robot.Intake;
import org.firstinspires.ftc.teamcode.robot.Launcher;
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
import dev.nextftc.hardware.controllable.RunToPosition;
import dev.nextftc.hardware.driving.DriverControlledCommand;

@Configurable
@TeleOp(name = "Driver Control")
public class DriverControlled extends NextFTCOpMode {
    public static double llDelay = 1.25;
    public static double speed = 0.6;

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
                // new PedroComponent(Constants::createFollower),
                new SubsystemComponent(Flywheel.INSTANCE, Intake.INSTANCE, Flicker.INSTANCE, Turret.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE);
    }

    @Override
    public void onUpdate() {
        /*
         * telemetry.addData("Far Launch Power", farLaunchPower);
         * telemetry.addData("Close Launch Power", closeLaunchPower);
         * telemetry.addData("Target X", angle);
         * telemetry.addData("Target Y", angleToGoal);
         * telemetry.addData("Distance from goal", distance_from_camera_to_target);
         * telemetry.update();
         * super.onUpdate();
         */

        /*
        telemetry.addData("Velocity RPM", Flywheel.INSTANCE.getVelocityRPM());
        telemetry.addData("Distance from goal inside subsystem", Flywheel.distanceToGoal); // Flywheel.INSTANCE.distance);
        telemetry.addData("Goal Velocity inside subsystem: ", Flywheel.launchVelocity); // goalVelocity);
        telemetry.addData("Encoder Value of Turret: ", Turret.INSTANCE.getEncoderValue());

        telemetry.update();
        super.onUpdate();

         */

    }

    @Override
    public void onInit() {
        Flywheel.powerState = false;
        Turret.powerState = false;
        Flicker.INSTANCE.allDown();

    }

    @Override
    public void onStartButtonPressed() {
        /*
         * Triangle (y)
         * Square (x)
         * Cross (a)
         * Circle (b)
         */
        // driverControlled.setScalar(speed);
        // driverControlled.schedule();

        Gamepads.gamepad1().rightTrigger().greaterThan(0.2)
                .whenBecomesTrue(() -> {
                    Flywheel.INSTANCE.shootOut().schedule();
                })
                .whenBecomesFalse(() -> {
                    Flywheel.INSTANCE.shutdown().schedule();
                });




        Gamepads.gamepad1().leftTrigger().greaterThan(0.2)
                .whenBecomesTrue(() -> {
                    Intake.INSTANCE.out().schedule();
                })
                .whenBecomesFalse(() -> {
                    Intake.INSTANCE.stop().schedule();
                });

        Gamepads.gamepad1().leftBumper()
                .whenBecomesTrue(() -> {
                    Intake.INSTANCE.in().schedule();
                })
                .whenBecomesFalse(() -> {
                    Intake.INSTANCE.stop().schedule();
                });

        Gamepads.gamepad1().dpadLeft()
                .whenBecomesTrue(() -> {
                    Flicker.INSTANCE.down1().schedule();
                });


        Gamepads.gamepad1().dpadRight()
                .whenBecomesTrue(() -> {
                    Flicker.INSTANCE.up1().schedule();
                });


        Gamepads.gamepad1().dpadUp()
                .whenBecomesTrue(() -> {
                    Flicker.INSTANCE.flick1().schedule();
                });

        Gamepads.gamepad1().x()
                .whenBecomesTrue(() -> {
                    Turret.INSTANCE.turnLeft().schedule();
                })
                .whenBecomesFalse(() -> {
                    Turret.INSTANCE.stop().schedule();
                });
        ;

        Gamepads.gamepad1().y()
                .whenBecomesTrue(() -> {
                    Turret.INSTANCE.turnRight().schedule();
                })
                .whenBecomesFalse(() -> {
                    Turret.INSTANCE.stop().schedule();
                });
        ;
    }
}