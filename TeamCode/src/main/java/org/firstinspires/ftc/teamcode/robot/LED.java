package org.firstinspires.ftc.teamcode.robot;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;

public class LED implements Subsystem {

    public static final LED INSTANCE = new LED();
    public final ServoEx led = new ServoEx("LED");

    public Command off(){return new SetPosition(led, 0).requires(this);}

    public Command on(){return new SetPosition(led, 1).requires(this);}
}
