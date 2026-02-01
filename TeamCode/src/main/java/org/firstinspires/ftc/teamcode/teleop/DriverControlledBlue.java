package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Team;
import org.firstinspires.ftc.teamcode.robot.ColorDetector;
import org.firstinspires.ftc.teamcode.robot.Flicker;
import org.firstinspires.ftc.teamcode.robot.Flywheel;
import org.firstinspires.ftc.teamcode.robot.Intake;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.LED;
import org.firstinspires.ftc.teamcode.robot.Limelight;
import org.firstinspires.ftc.teamcode.pedroPathing.PoseStorage;
import org.firstinspires.ftc.teamcode.robot.Turret;
import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.extensions.pedro.PedroDriverControlled;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.DriverControlledCommand;

@Configurable
@TeleOp(name = "DriverControlledBlue")
public class DriverControlledBlue extends NextFTCOpMode {
    public static boolean turret = true;
    public static double speed = 0.7;

    private DriverControlledCommand driverControlled = new PedroDriverControlled(
            Gamepads.gamepad1().leftStickY().negate(),
            Gamepads.gamepad1().leftStickX().negate(),
            Gamepads.gamepad1().rightStickX().negate());


    public DriverControlledBlue() {
        addComponents(
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(Flywheel.INSTANCE, Intake.INSTANCE, Flicker.INSTANCE, Turret.INSTANCE, Limelight.INSTANCE, ColorDetector.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE);
    }

    @Override
    public void onUpdate() {
        follower().update();
        PoseStorage.setPose(follower().getPose());
        telemetryUpdate();
        super.onUpdate();

    }

    private void telemetryUpdate(){
        telemetry.addData("Current Pose: ", PoseStorage.getPose());
        telemetry.addData("Current Angle: ", ((PoseStorage.getHeadingDegrees())));
        telemetry.addData("Turret pos: ", Turret.INSTANCE.getDegrees());
        telemetry.addData("Turret encoders: ", Turret.INSTANCE.getEncoderValue());
        telemetry.addData("goal angle: ", Turret.goalAngle);
        telemetry.addData("angle need to turn: ", Turret.INSTANCE.getMoveAngle());
        telemetry.addData("locked", Turret.locked);
        telemetry.addData("Sensor Values: ", ColorDetector.INSTANCE.getSensorValues());
        telemetry.addData("current rpm", Flywheel.INSTANCE.getVelocityRPM());
        telemetry.update();
    }

    @Override
    public void onInit() {
        Flywheel.powerState = false;
        Turret.powerState = false;
        Turret.locked = false;
        Turret.INSTANCE.setEncoderValue(0);
        //PoseStorage.resetPose();
        follower().setStartingPose(PoseStorage.getPose());
        follower().update();
        Team.setTeam(0);
        telemetryUpdate();
        //Turret.INSTANCE.zero();

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
        Flicker.INSTANCE.flickThreeBalls().schedule();
        Turret.INSTANCE.autoAlignTrig().schedule();

        Gamepads.gamepad1().rightTrigger().greaterThan(0.2)
                .whenBecomesTrue(() -> {
                    Flywheel.INSTANCE.shootOut().schedule();
                })
                .whenBecomesFalse(() -> {
                    Flywheel.INSTANCE.shutdown().schedule();
                });

        Gamepads.gamepad1().rightBumper()
                .whenBecomesTrue(() -> {
                    Flywheel.INSTANCE.constantShot(1200).schedule();
                })
                .whenBecomesFalse(() -> {
                    Flywheel.INSTANCE.shutdown().schedule();
                    Flicker.INSTANCE.allDown().schedule();
                });


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
                .whenBecomesTrue(() -> {
                    Intake.INSTANCE.out().schedule();
                })
                .whenBecomesFalse(() -> {
                    Intake.INSTANCE.stop().schedule();
                });

        Gamepads.gamepad1().a().toggleOnBecomesTrue()
                .whenBecomesTrue(() -> {
                    driverControlled.setScalar(speed);
                })
                .whenBecomesFalse(() -> {
                    driverControlled.setScalar(1);
                });


        Gamepads.gamepad1().y()
                .whenBecomesTrue(() -> {
                    Flicker.INSTANCE.flick2().schedule();
                });

        Gamepads.gamepad1().b()
                .whenBecomesTrue(() -> {
                    Flicker.INSTANCE.flick3().schedule();
                });

        Gamepads.gamepad1().x()
                .whenBecomesTrue(() -> {
                    Flicker.INSTANCE.flick1().schedule();
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

        Gamepads.gamepad1().dpadUp().toggleOnBecomesTrue()
                .whenBecomesTrue(() -> {
                    Turret.INSTANCE.autoAlignPerpetual.cancel();
                })
                .whenBecomesFalse(() -> {
                    Turret.INSTANCE.autoAlignPerpetual.schedule();
                });

        Gamepads.gamepad1().dpadDown()
                .whenBecomesTrue(() -> {
                    Flywheel.INSTANCE.reverse().schedule();
                })
                .whenBecomesFalse(() -> {
                    Flywheel.INSTANCE.shutdown().schedule();
                });

        Gamepads.gamepad1().leftStickButton()
                .whenBecomesTrue(() -> {
                    follower().setStartingPose(new Pose(72,72,Math.toRadians(90)));
                    PoseStorage.setPose(new Pose(72,72,Math.toRadians(90)));
                });

    }
}