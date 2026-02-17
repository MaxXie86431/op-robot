package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.configurables.annotations.Configurable;
import org.firstinspires.ftc.teamcode.robot.ColorDetector;
import org.firstinspires.ftc.teamcode.robot.Flicker;
import org.firstinspires.ftc.teamcode.robot.Flywheel;
import org.firstinspires.ftc.teamcode.robot.Intake;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.Limelight;
import org.firstinspires.ftc.teamcode.pedroPathing.PoseStorage;
import org.firstinspires.ftc.teamcode.robot.Turret;
import org.firstinspires.ftc.teamcode.utils.GenetonUtils;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.extensions.pedro.PedroDriverControlled;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.DriverControlledCommand;

@Configurable
public abstract class DriverControlled extends NextFTCOpMode {
    public static boolean turret = true;
    public static double speed = 0.7;

    private DriverControlledCommand driverControlled = new PedroDriverControlled(
            Gamepads.gamepad1().leftStickY().negate(),
            Gamepads.gamepad1().leftStickX().negate(),
            Gamepads.gamepad1().rightStickX().negate());


    public DriverControlled() {
        addComponents(
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(Flywheel.INSTANCE, Intake.INSTANCE, Flicker.INSTANCE, Turret.INSTANCE, Limelight.INSTANCE),//, ColorDetector.INSTANCE,
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE);
    }

    /**
     * Initialize team-specific settings. Called in onInit().
     */
    protected abstract void initTeam();

    @Override
    public void onUpdate() {
        follower().update();
        PoseStorage.setPose(follower().getPose());
        Flywheel.INSTANCE.setLaunchVelocity(GenetonUtils.INSTANCE.getTargetVelocity());
        telemetryUpdate();
        super.onUpdate();
    }

    private void telemetryUpdate(){
        telemetry.addData("Heading Angle: ", PoseStorage.getHeadingDegrees());
        telemetry.addData("Goal Angle: ", GenetonUtils.INSTANCE.getFieldTargetAngle());
        telemetry.addData("Target Turret Angle: ", GenetonUtils.INSTANCE.getTargetTurretAngle());
        telemetry.addData("Current Turret Angle: ", Turret.INSTANCE.getDegrees());

        telemetry.addData("current RPM", Flywheel.INSTANCE.getVelocityRPM());
        telemetry.addData("target RPM: ", Flywheel.launchVelocity);
        telemetry.addData("target distance: ", Flywheel.distanceToGoal);

        telemetry.addData("Current Pose: ", PoseStorage.getPose());

        telemetry.addData("Current Angle: ", ((PoseStorage.getHeadingDegrees())));
        telemetry.addData("Turret pos: ", Turret.INSTANCE.getDegrees());
        telemetry.addData("Turret encoders: ", Turret.INSTANCE.getEncoderValue());
        telemetry.addData("goal angle: ", GenetonUtils.INSTANCE.getFieldTargetAngle());
        telemetry.addData("angle need to turn: ", GenetonUtils.INSTANCE.getTargetTurretAngle());

        telemetry.addData("LimeLight Yaw: ", Limelight.INSTANCE.calculateAlignmentAngle());
        telemetry.addData("LimeLight Offset: ", GenetonUtils.LIME_LIGHT_OFFSET);//.INSTANCE.calculateAlignmentAngle());

        //telemetry.addData("Sensor Values: ", ColorDetector.INSTANCE.getSensorValues());

        telemetry.update();
    }

    @Override
    public void onInit() {
        Flywheel.powerState = false;
        Turret.powerState = false;
        Flicker.INSTANCE.setFlickDelay(Flicker.flickDelayTeleOp);
        //PoseStorage.resetPose();
        follower().setStartingPose(PoseStorage.getPose());
        follower().update();
        initTeam();
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


        Gamepads.gamepad1().rightTrigger().greaterThan(0.2)
                .whenBecomesTrue(() -> {
                    Flywheel.INSTANCE.shootOut().schedule();
                    Turret.INSTANCE.autoAlignPerpetual.schedule();
                })
                .whenBecomesFalse(() -> {
                    Flywheel.INSTANCE.shutdown().schedule();
                    Flicker.INSTANCE.allDown().schedule();
                    Turret.INSTANCE.autoAlignPerpetual.cancel();
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

        Gamepads.gamepad1().b()
                .whenBecomesTrue(() -> {
                    Flywheel.INSTANCE.reverse().schedule();
                })
                .whenBecomesFalse(() -> {
                    Flywheel.INSTANCE.shutdown().schedule();
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


        //separation

        Gamepads.gamepad2().rightTrigger().greaterThan(0.2)
                .whenBecomesTrue(() -> {
                    Flywheel.INSTANCE.shootOut().schedule();
                    Turret.INSTANCE.autoAlignPerpetual.schedule();
                })
                .whenBecomesFalse(() -> {
                    Flywheel.INSTANCE.shutdown().schedule();
                    Flicker.INSTANCE.allDown().schedule();
                    Turret.INSTANCE.autoAlignPerpetual.cancel();
                });

        Gamepads.gamepad2().rightBumper()
                .whenBecomesTrue(() -> {
                    Flywheel.INSTANCE.endGameShot().schedule();
                })
                .whenBecomesFalse(() -> {
                    Flywheel.INSTANCE.shutdown().schedule();
                    Flicker.INSTANCE.allDown().schedule();
                });


        Gamepads.gamepad2().leftTrigger().greaterThan(0.2)
                .whenBecomesTrue(() -> {
                    Intake.INSTANCE.in().schedule();
                })
                .whenBecomesFalse(() -> {
                    new ParallelGroup(
                            Intake.INSTANCE.stop(),
                            Flicker.INSTANCE.flickdown1()
                    ).schedule();
                });

        Gamepads.gamepad2().leftBumper()
                .whenBecomesTrue(() -> {
                    Intake.INSTANCE.out().schedule();
                })
                .whenBecomesFalse(() -> {
                    Intake.INSTANCE.stop().schedule();
                });



        Gamepads.gamepad2().dpadLeft()
                .whenBecomesTrue(() -> {
                    Turret.INSTANCE.turnLeft().schedule();
                })
                .whenBecomesFalse(() -> {
                    Turret.INSTANCE.stop().schedule();
                });
        Gamepads.gamepad2().dpadRight()
                .whenBecomesTrue(() -> {
                    Turret.INSTANCE.turnRight().schedule();
                })
                .whenBecomesFalse(() -> {
                    Turret.INSTANCE.stop().schedule();
                });


        Gamepads.gamepad2().x()
                .whenBecomesTrue(() -> {
                    Flicker.INSTANCE.flick1Switch().schedule();
                });

        Gamepads.gamepad2().y()
                .whenBecomesTrue(() -> {
                    Flicker.INSTANCE.flick2Switch().schedule();
                });

        Gamepads.gamepad2().b()
                .whenBecomesTrue(() -> {
                    Flicker.INSTANCE.flick3Switch().schedule();
                });

        Gamepads.gamepad1().touchpad()
                .whenBecomesTrue(()->{
                    Turret.INSTANCE.tuneTurret(gamepad1).schedule();
                });
    }
}
