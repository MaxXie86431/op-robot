package org.firstinspires.ftc.teamcode.robot;

import com.bylazar.configurables.annotations.Configurable;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.commands.utility.LambdaCommand;
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

    public double getEncoderValue(){
        return turretMotor.getCurrentPosition();
    }

    public void zeroEncoderValue() {
        turretMotor.setCurrentPosition(0);
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

    public Command autoTrack = new LambdaCommand()
    .setStart(() -> {
        powerState = true;
    })
    .setUpdate(() -> {
        double angle = Limelight.INSTANCE.calculateAlignmentAngle();
        if (angle != 0) {
            double targetPosition = turretMotor.getCurrentPosition() + angle * positionPerDegree;
            new RunToPosition(controller, targetPosition).schedule();
        }
    })
    .setStop(interrupted -> {
    })
    .setIsDone(() -> false)
    .requires(this)
    .setInterruptible(true);

    @Override
    public void periodic() {
        if(powerState && getEncoderValue() > -1000 && getEncoderValue() < 1700) {
            turretMotor.setPower(controller.calculate(turretMotor.getState()));
        }
        else {

        }
    }
}
