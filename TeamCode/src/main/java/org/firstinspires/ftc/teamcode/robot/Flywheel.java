package org.firstinspires.ftc.teamcode.robot;

import com.bylazar.configurables.annotations.Configurable;

import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelDeadlineGroup;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.commands.utility.NullCommand;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.powerable.SetPower;
import dev.nextftc.core.commands.utility.InstantCommand;

@Configurable
public class Flywheel implements Subsystem{
    public static double kP = 0.005;
    public static double kI = 0.0175;
    public static double kD = 0.02;
    public static double kV = 0.025;
    public static double kA = 0.02;
    public static double kS = 0.03;
    public static double distanceToGoal = 0;
    public static double launchVelocity = 0;

    public static boolean powerState = false;

    private ControlSystem controller = ControlSystem.builder()
            .velPid(kP, kI, kD)
            .basicFF(kV, kA, kS)
            .build();

    public static final Flywheel INSTANCE = new Flywheel();
    private Flywheel() { }
    private MotorEx motor;
    public static int inVelocity = -1000;
    public static double launchBuffer = 2;
    public static double launchPower;
    private double currentTargetVelocity = 0;

    @Override
    public void initialize() {
        launchPower = 0.8;
        motor = new MotorEx("Flywheel").reversed();
    }
    public double getVelocityRPM(){
        double ticksPerSecond = motor.getVelocity();
        /*double revPerSec = ticksPerSecond / TICKS_PER_REVOLUTION;
        double degreesPerSecond = revPerSec * 360;*/
        return ticksPerSecond;
    }

    public Command out(double velocity) {
        //double ticksPerSecond = velocity * TICKS_PER_REVOLUTION / 60.0;
        return new SequentialGroup(
                new InstantCommand(() -> powerState = true),
                new RunToVelocity(controller, velocity, 20)
        ).requires(this);
    }

    public Command shutdown(){
        return new SequentialGroup(
                new InstantCommand(() -> powerState = false),
                new SetPower(motor, 0)
        ).requires(this);
    }

    public Command reverse() {
        return new ParallelGroup(
                new RunToVelocity(controller,inVelocity, 20).requires(this)
        );
    }

    public Command shootOut() {
        //double[] values = Limelight.INSTANCE.calculateLaunchPower();
        //distanceToGoal = values[0];
        launchVelocity = 1100;
        if (launchVelocity==0) {
            return new NullCommand();
        }
        return new SequentialGroup(
                out(launchVelocity),
                new ParallelDeadlineGroup(
                        new Delay(launchBuffer)
                )

        ).requires(this);
    }

    public Command constantShot(int velocity) {
        return new SequentialGroup(
                out(velocity)
        ).requires(this);
    }

    public Command outPower() {
        //double ticksPerSecond = velocity * TICKS_PER_REVOLUTION / 60.0;
        return new SequentialGroup(
                new InstantCommand(() -> powerState = true),
                new SetPower(motor, launchPower)
        ).requires(this);
    }

    @Override
    public void periodic() {
        if(powerState)
            motor.setPower(controller.calculate(motor.getState()));
    }

    public Command increasePower() {
        return new InstantCommand(() -> {
            if (launchPower<1) {
                launchPower += 0.025;
            } // change field here
        });
    }

    public Command decreasePower() {
        return new InstantCommand(() -> {
            if (launchPower>0.05) {
                launchPower -= 0.025;
            }
        });
    }


}