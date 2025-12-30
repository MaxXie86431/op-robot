package org.firstinspires.ftc.teamcode.robot;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;

@Configurable
public class Flicker implements Subsystem {

    public static final Flicker INSTANCE = new Flicker();
    private final ServoEx servo1= new ServoEx("Flicker1");
    private final ServoEx servo2= new ServoEx("Flicker2");
    private final ServoEx servo3= new ServoEx("Flicker3");
    public static double pos = 0.5;

    public Command down(ServoEx servo) {
        return  new SetPosition(servo, 0);
    }
    public Command up(ServoEx servo) {
        return new SetPosition(servo, 1);
    }

    public Command up1() {
        return new SetPosition(servo1, pos);
    }

    public Command down1() {
        return new SetPosition(servo1, 0);
    }

    public Command up2() {
        return new SetPosition(servo2, 1);
    }

    public Command down2() {
        return new SetPosition(servo2, 0);
    }

    public Command flick1() {
        return new SequentialGroup(
                up(servo1),
                down(servo1)
        );
    }

    public Command flick2() {
        return new SequentialGroup(
                up(servo2),
                down(servo2)
        );
    }

    public Command flick3() {
        return new SequentialGroup(
                up(servo2),
                down(servo2)
        );
    }

    public Command allDown() {
        return new ParallelGroup(
              down(servo1),
                down(servo2),
                down(servo3)
                );
    }

}