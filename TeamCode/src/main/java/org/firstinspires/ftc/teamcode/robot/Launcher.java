package org.firstinspires.ftc.teamcode.robot;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.time.Duration;

import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.positionable.SetPosition;
import dev.nextftc.hardware.powerable.SetPower;

@Configurable
public class Launcher implements Subsystem {

    public static final Launcher INSTANCE = new Launcher();
    private Launcher() {
    }
    private MotorEx motor;
    public static double closeLaunchPower;
    public static double farLaunchPower;

    @Override
    public void initialize() {
        motor = new MotorEx("BottomLaunch");
        closeLaunchPower = 0.8;
        farLaunchPower=1;
    }
    public Command inward() {
        return new ParallelGroup(
                new SetPower(motor, -1)
        ).requires(this);
    }
    public Command outward(double power, double delay) {
        return new SequentialGroup(
                new SetPower(motor, power),
                new Delay(delay)
        ).requires(this);
        /*
        return new SequentialGroup(
            new ParallelGroup(
                new ParallelGroup(
                        new SetPower(motor1, power),
                        new SetPower(motor2, power)
                ),
                new SequentialGroup(
                    new Delay(delay),
                    Intermediate.INSTANCE.rollup()
                )
            )
        ).requires(this);
         */
        //return new SetPower(motor, power).requires(this);
    }

    public Command rawLaunch(double power) {
        return new SetPower(motor,power).requires(this);
    }
    public Command stop(){
        return new ParallelGroup(
                new SetPower(motor, 0)
        ).requires(this);
    }

    public Command increaseFarPower() {

        return new InstantCommand(() -> {
            if (farLaunchPower<1) {
                farLaunchPower += 0.025;
            } // change field here
        });
    }

    public Command decreaseFarPower() {
        return new InstantCommand(() -> {
            if (farLaunchPower>0.05) {
                farLaunchPower -= 0.025;
            }
        });
    }

    public Command increaseClosePower() {

        return new InstantCommand(() -> {
            if (closeLaunchPower<1) {
                closeLaunchPower += 0.025;
            } // change field here
        });
    }
    public Command decreaseClosePower() {
        return new InstantCommand(() -> {
            if (closeLaunchPower>0.05) {
                closeLaunchPower -= 0.025;
            }
        });
    }

    public double getFarLaunchPower() {
        return farLaunchPower;
    }

}