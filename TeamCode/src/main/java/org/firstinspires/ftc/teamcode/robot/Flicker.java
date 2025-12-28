package org.firstinspires.ftc.teamcode.robot;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;

public class Flicker implements Subsystem {

    public static final Flicker INSTANCE = new Flicker();
    private final ServoEx servo= new ServoEx("Flicker-2-3");;

    public Command closeServo = new SetPosition(servo, 0).requires(this);
    public Command openServo = new SetPosition(servo, 0.8).requires(this);


}