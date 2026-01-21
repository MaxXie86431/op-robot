package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Flicker;
import org.firstinspires.ftc.teamcode.robot.Flywheel;
import org.firstinspires.ftc.teamcode.robot.Intake;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.Limelight;
import org.firstinspires.ftc.teamcode.pedroPathing.PoseStorage;
import org.firstinspires.ftc.teamcode.robot.Turret;
import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.extensions.pedro.PedroDriverControlled;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.DriverControlledCommand;

@Configurable
@TeleOp(name = "turret test")
public class turrettest extends NextFTCOpMode {
    //left is up right is down rn
    public static double llDelay = 1.25;
    public static boolean turret = true;
    private static double tagPos = 45;
    public static double testDegree = 135;

    private DriverControlledCommand driverControlled = new PedroDriverControlled(
            Gamepads.gamepad1().leftStickY().negate(),
            Gamepads.gamepad1().leftStickX().negate(),
            Gamepads.gamepad1().rightStickX().negate());


    public turrettest() {
        addComponents(
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(Flywheel.INSTANCE, Intake.INSTANCE, Flicker.INSTANCE, Turret.INSTANCE, Limelight.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE);
    }

    @Override
    public void onUpdate() {
        PoseStorage.setPose(follower().getPose());

        telemetry.addData("Current Pose: ", PoseStorage.getPose());
        telemetry.addData("Current Angle: ", ((PoseStorage.getPose().getHeading())*180.0/Math.PI));
        telemetry.addData("Turret pos: ", Turret.INSTANCE.getDegrees());
        telemetry.addData("Turret encoders: ", Turret.INSTANCE.getEncoderValue());
        telemetry.addData("goal angle: ", Turret.goalAngle);
        telemetry.addData("angle need to turn: ", Turret.angle);
        telemetry.addData("locked", Turret.locked);

        telemetry.update();


    }

    @Override
    public void onInit() {
        Flywheel.powerState = false;
        Turret.powerState = false;
        Turret.locked = false;
        Flicker.INSTANCE.allDown();
        PoseStorage.resetPose();
        follower().setStartingPose(PoseStorage.getPose());
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
        Turret.INSTANCE.autoAlignPerpetual.schedule();


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
                    Flicker.INSTANCE.flick1().schedule();
                });


        Gamepads.gamepad1().y()
                .whenBecomesTrue(() -> {
                    Flicker.INSTANCE.flick2().schedule();
                });

        Gamepads.gamepad1().b()
                .whenBecomesTrue(() -> {
                    Flicker.INSTANCE.flick3().schedule();
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
                    Turret.INSTANCE.autoTrackButton().schedule();
                });




    }
}