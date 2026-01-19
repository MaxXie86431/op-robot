package org.firstinspires.ftc.teamcode.robot;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.internal.hardware.android.GpioPin;
import org.firstinspires.ftc.teamcode.pedroPathing.PoseStorage;
import org.firstinspires.ftc.teamcode.teleop.DriverControlled;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.commands.utility.NullCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.controllable.RunToPosition;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;

@Configurable
public class Turret implements Subsystem {
    //-1000 1700
    public static double kP = 0.005;
    public static double kI = 0;
    public static double kD = 0;
    public static double kV = 0.025;
    public static double kA = 0.02;
    public static double kS = 0.03;
    public static double velocity = 0.5;
    public static double ticksPerRev = 753.2;
    public static double rotationRatio = 0.22;
    public static double positionPerDegree = 9.51;
    public static boolean powerState = false;
    public static int tolerance= 50;

    private final ControlSystem controller = ControlSystem.builder()
            .posPid(kP, kI, kD)
            //.basicFF()
            .basicFF(kV, kA, kS)
            .build();

    public static final Turret INSTANCE = new Turret();

    private MotorEx turretMotor;

    private Turret() {}

    @Override
    public void initialize() {
        turretMotor = new MotorEx("Turret-Gear").reversed();
    }

    public Command turnRight() {
        return new SequentialGroup(
                new InstantCommand(() -> powerState = false),
                new SetPower(turretMotor, velocity)
        ).requires(this);
    }

    public Command turnLeft(){
        return new SequentialGroup(
            new InstantCommand(() -> powerState = false),
            new SetPower(turretMotor, -1 * velocity)
        ).requires(this);
    }

    public Command stop(){
        return new SequentialGroup(
            new InstantCommand(() -> powerState = false),
            new SetPower(turretMotor, 0)
        ).requires(this);
    }

    public Command goToZero(){
        return new SequentialGroup(
                new InstantCommand(() -> powerState = true),
                new RunToPosition(controller, 0)
        ).requires(this);
    }

    public Command turnByDegrees(double degrees){
        return new SequentialGroup(
            new InstantCommand(() -> powerState = true),
            new RunToPosition(controller, turretMotor.getCurrentPosition()  + degrees * positionPerDegree)
        ).requires(this);
    }

    public Command turnToDegrees(double degrees){
        return new SequentialGroup(
                new InstantCommand(() -> powerState = true),
                new RunToPosition(controller, degrees * positionPerDegree)
        ).requires(this);
    }

    public Command autoAlign(double tagPos, double heading) {
        //double turretPos = -1 * getDegrees();
        //double angle = (heading-tagPos) + (turretPos-90);
        double angle = tagPos-heading+getDegrees();


        ActiveOpMode.telemetry().addData("angle need to turn:", angle);
        ActiveOpMode.telemetry().addData("current encoder degrees: ", getDegrees());
        ActiveOpMode.telemetry().addData("Current pose: ", PoseStorage.getPose());


        return new SequentialGroup(
                new InstantCommand(() -> powerState = true),
                turnByDegrees(-angle)
        ).requires(this);

    }


    public double getEncoderValue(){
        return turretMotor.getCurrentPosition();
    }
    public double getDegrees() {
        return getEncoderValue()/positionPerDegree;
    }


    public void setEncoderValue(int pos) {
        turretMotor.setCurrentPosition(pos);
    }

    /*
    public Command autoTrack() {
        double angle = Limelight.INSTANCE.calculateAlignmentAngle();
        return new SequentialGroup(
                new InstantCommand(() -> powerState = true),
                new RunToPosition(controller, turretMotor.getCurrentPosition()  + angle * positionPerDegree)
        ).requires(this);
    }
    */

    public Command autoTrackButton() {
        double angle = Limelight.INSTANCE.calculateAlignmentAngle();
        if (angle != 0) {
            return new SequentialGroup(
                    new InstantCommand(() -> powerState = true),
                    turnByDegrees(angle)
            );

        }
        return new NullCommand();
    }
    public Command autoTrack = new LambdaCommand()
    .setStart(() -> {
        powerState = true;
    })
    .setUpdate(() -> {
        if (DriverControlled.turret) {
            double angle = Limelight.INSTANCE.calculateAlignmentAngle();
            if (angle != 0) {
                double targetPosition = controller.getLastMeasurement().getPosition() + angle * positionPerDegree;
                new RunToPosition(controller, targetPosition).schedule();
            }
        }
    })
    .setStop(interrupted -> {
    })
    .setIsDone(() -> false)
    .requires(this)
    .setInterruptible(true);

    @Override
    public void periodic() {
        if(powerState) {
            turretMotor.setPower(controller.calculate(turretMotor.getState()));
        }


    }
}
