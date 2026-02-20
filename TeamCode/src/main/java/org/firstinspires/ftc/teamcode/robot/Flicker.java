package org.firstinspires.ftc.teamcode.robot;

import com.bylazar.configurables.annotations.Configurable;

import java.util.ArrayList;
import java.util.List;

import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.utility.NullCommand;
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
    public static double flickDelay = 0.8;
    public static double flickDelayTeleOp = 0.8;
    public static double flickDelayAuto = 0.8;
    public static double betweenflicksDelay = 0.09;
    public static double betweenflicksDelayAuto = 0.09;
    private static String flicked = "";
    /*
    public static double up1 = 0.4;
    public static double down1 = 0.03;
    public static double up2 = 0.6;
    public static double down2 = 0.94;
    public static double up3 = 0.5;
    public static double down3 = 0.075;
    */
    public static boolean flick1Up = false;
    public static boolean flick2Up = false;
    public static boolean flick3Up = false;
    public static double up1 = 0.5;
    public static double down1 = 0.14;
    public static double up2 = 0.55;
    public static double down2 = 0.91;
    public static double up3 = 0.7;
    public static double down3 = 0.08;
    public static double bumper = 0.17;

    public Command flick1() {
        return new SequentialGroup(
                new SetPosition(servo1, up1),
                new Delay(flickDelay),
                new SetPosition(servo1, down1)
        );
    }

    public Command flick2() {
        return new SequentialGroup(
                new SetPosition(servo2, up2),
                new Delay(flickDelay),
                new SetPosition(servo2, down2)
        );
    }

    public Command flick3() {
        return new SequentialGroup(
                new SetPosition(servo3, up3),
                new Delay(flickDelay),
                new SetPosition(servo3, down3)
        );
    }

    public Command intakeHelper() {
        return new SetPosition(servo1, bumper);
    }

    public Command flick1Switch() {
        if(flick1Up)
            return flickdown1();
        else
            return flickup1();
    }

    public Command flick2Switch() {
        if(flick2Up)
            return flickdown2();
        else
            return flickup2();
    }

    public Command flick3Switch() {
        if(flick3Up)
            return flickdown3();
        else
            return flickup3();
    }

    public Command flickup1() {
        flick1Up = true;
        return new SequentialGroup(
                new SetPosition(servo1, up1)
        );
    }

    public Command flickdown1() {
        flick1Up = false;
        return new SequentialGroup(
                new SetPosition(servo1, down1)
        );
    }

    public Command flickup2() {
        flick2Up = true;
        return new SequentialGroup(
                new SetPosition(servo2, up2)
        );
    }

    public Command flickdown2() {
        flick2Up = false;
        return new SequentialGroup(
                new SetPosition(servo2, down2)
        );
    }

    public Command flickup3() {
        flick3Up = true;
        return new SequentialGroup(
                new SetPosition(servo3, up3)
        );
    }

    public Command flickdown3() {
        flick3Up = false;
        return new SequentialGroup(
                new SetPosition(servo3, down3)
        );
    }

    public Command chooseFlick(int pos) {
        if (pos == 1) {
            return flick1();
        }
        else if (pos == 2) {
            return flick2();
        }
        else if (pos == 3) {
            return flick2();
        }
        else {
            return new NullCommand();
        }
    }

    public Command allDown() {
        return new ParallelGroup(
                new SetPosition(servo1, down1),
                new SetPosition(servo2, down2),
                new SetPosition(servo3, down3)
        );
    }

    public Command flickThreeBalls() {
        return new SequentialGroup(
                flick3(),
                new Delay(betweenflicksDelay),
                flick1(),
                new Delay(betweenflicksDelay),
                flick2(),
                new Delay(betweenflicksDelay)
        );
    }

    public Command flickThreeBallsAuto() {
        return new SequentialGroup(
                flick3(),
                new Delay(betweenflicksDelayAuto),
                flick2(),
                new Delay(betweenflicksDelayAuto),
                flick1(),
                new Delay(betweenflicksDelayAuto)
        );
    }

    public void setFlickDelay(double delay) {
        flickDelay = delay;
    }

}