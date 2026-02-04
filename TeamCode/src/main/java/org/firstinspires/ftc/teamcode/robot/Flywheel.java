package org.firstinspires.ftc.teamcode.robot;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

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

    //PIDF Constants
    public static double kP = 0.005;
    public static double kI = 0;
    public static double kD = 0.01;
    public static double kV = 0.0005;
    public static double kA = 0.05;
    public static double kS = 0.05;
    private double lastKP, lastKI, lastKD, lastKV, lastKA, lastKS; //Enabled for tuning PIDF

    public static double distanceToGoal = 0;
    public static double launchVelocity = 0;
    public static int tolerance = 30;

    public static boolean powerState = false;

    private final ControlSystem controller = ControlSystem.builder()
            .velPid(kP, kI, kD)
            .basicFF(kV, kA, kS)
            .build();

    public static final Flywheel INSTANCE = new Flywheel();
    private Flywheel() { }
    private final MotorEx motor = new MotorEx("Flywheel");
    public static int inVelocity = -1000;
    public static double launchPower;

    @Override
    public void initialize() {
        launchPower = 0.55;
    }

    public double getVelocityRPM(){
        return motor.getVelocity();
    }

    public Command out(double velocity) {
        //double ticksPerSecond = velocity * TICKS_PER_REVOLUTION / 60.0;
        return new SequentialGroup(
                new InstantCommand(() -> powerState = true),
                new RunToVelocity(controller, velocity, tolerance)
        ).requires(this);
    }

    private final ElapsedTime rampTimer = new ElapsedTime();

    public Command rampedOut(double targetVelocity) {
        return new SequentialGroup(
                out(targetVelocity * 0.4), // 1. Start at 40% speed (e.g. ~600 RPM)
                new Delay(0.7),            // 2. Let it stabilize
                out(targetVelocity * 0.7), // 3. Bump to 70% (e.g. ~1000 RPM)
                new Delay(0.7),            // 4. Let it stabilize
                out(targetVelocity)        // 5. Final target of 1490 RPM
        );
    }

    public Command smoothRamp(double targetVelocity, double durationSeconds) {
        return new SequentialGroup(
                // 1. Reset the timer at the very beginning
                new InstantCommand(() -> {
                    powerState = true;
                    rampTimer.reset();
                }),
                // 2. Use a Deadline Group to force the loop to end after durationSeconds
                new ParallelDeadlineGroup(
                        new Delay(durationSeconds), // The "Deadline" - group stops when this finishes
                        new LambdaCommand()
                                .setUpdate(() -> {
                                    double elapsed = rampTimer.seconds();
                                    double progress = Math.min(elapsed / durationSeconds, 1.0);
                                    double currentTarget = progress * targetVelocity;

                                    // Update the motor target every loop
                                    out(currentTarget).schedule();
                                })
                                .perpetually() // Tell the Lambda to run until the Deadline stops it
                )
        ).requires(this);
    }

    public Command regress() {
        return new SequentialGroup(
                new InstantCommand(() -> powerState = true),
                new RunToVelocity(controller, getPower() * 2000, 20)
        ).requires(this);
    }

    public Command shutdown(){
        return new SequentialGroup(
                new InstantCommand(() -> {
                    powerState = false;
                }
                ),

                new SetPower(motor, 0)
        ).requires(this);
    }

    public Command reverse() {
        return new ParallelGroup(
                new RunToVelocity(controller,inVelocity, 20).requires(this)
        );
    }

    public Command shootOut() {
        /*
        double[] values = Limelight.INSTANCE.calculateLaunchPower();
        distanceToGoal = values[0];
        launchVelocity = values[1];

         */
        return new SequentialGroup(
                Turret.INSTANCE.autoTrackButton(),
                Turret.INSTANCE.turnByDegrees(-3),
                new InstantCommand(() -> {
                    double[] values = Limelight.INSTANCE.calculateLaunchPower();
                    distanceToGoal = values[0];
                    launchVelocity = values[1];
                }),
                out(launchVelocity),
                new ParallelDeadlineGroup(
                        new Delay(0.00265*launchVelocity-1.2),
                        out(launchVelocity)
                ),
                Flicker.INSTANCE.flickThreeBalls()
        );
    }

    public Command constantShot(int velocity) {
        return new SequentialGroup(
                new ParallelDeadlineGroup(
                        new Delay(0.00265*velocity-1.2),
                        out(velocity)
                ),
                Flicker.INSTANCE.flickThreeBalls()
        );
    }

    public Command outPower(double power) {
        //double ticksPerSecond = velocity * TICKS_PER_REVOLUTION / 60.0;
        return new SequentialGroup(
                new InstantCommand(() -> powerState = false),
                new SetPower(motor, power)
        );
    }

    @Override
    public void periodic() {
        /*
        // 1. Check if any value from Panels has changed
        if (kP != lastKP || kI != lastKI || kD != lastKD || kV != lastKV || kA != lastKA || kS != lastKS) {

            // 2. Rebuild the controller with the NEW values
            controller = ControlSystem.builder()
                    .velPid(kP, kI, kD)
                    .basicFF(kV, kA, kS)
                    .build();

            // 3. Update 'last' values so we don't rebuild every single loop
            lastKP = kP; lastKI = kI; lastKD = kD;
            lastKV = kV; lastKA = kA; lastKS = kS;
        }
        */
        if (powerState) {
            motor.setPower(controller.calculate(motor.getState()));
        }
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
            if (launchPower>0.025) {
                launchPower -= 0.025;
            }
        });
    }

    public double getPower() {
        return launchPower;
    }


}