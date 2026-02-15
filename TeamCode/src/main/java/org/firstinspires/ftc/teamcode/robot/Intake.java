package org.firstinspires.ftc.teamcode.robot;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.utility.NullCommand;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;
@Configurable
public class Intake implements Subsystem {

    public static final Intake INSTANCE = new Intake();

    private static final MotorEx intakeMotor = new MotorEx("IntakeMotor");
    public static double intakeDelay = 0.1;

    private Intake() {}

    public Command in(){
        /*
        if (!ColorDetector.INSTANCE.hasOpenSlots()){
            return new NullCommand();
        }
        */
        return new SequentialGroup(
               Flicker.INSTANCE.intakeHelper(),
               new Delay(intakeDelay),
               new SetPower(intakeMotor, 1)
        );

    }

    public Command out(){

        return new SetPower(intakeMotor, -0.7).requires(this);
    }

    public Command stop(){
        return new SetPower(intakeMotor, 0).requires(this);
    }

}