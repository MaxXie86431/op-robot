package org.firstinspires.ftc.teamcode.robot;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import java.util.ArrayList;
import java.util.List;

import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.utility.NullCommand;
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
    public static double flickDelay = 1;
    private static String flicked = "";
    public static double up1 = 0.55;
    public static double down1 = 0.03;
    public static double up2 = 0.32;
    public static double down2 = 0.94;
    public static double up3 = 0.7;
    public static double down3 = 0.95;
    public static boolean flick1Up = false;
    public static boolean flick2Up = false;
    public static boolean flick3Up = false;


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

    public Command flick1Switch(){
        if(flick1Up)
            return flickdown1();
        else
            return flickup1();
    }

    public Command flick2Switch(){
        if(flick2Up)
            return flickdown2();
        else
            return flickup2();
    }

    public Command flick3Switch(){
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

    public Command flickAll() {
        String motif = Limelight.INSTANCE.color();
        String inventory = ColorDetector.INSTANCE.getSensorValues();
        List<Command> flicks= new ArrayList<>();

        for (int i = 0; i <= 2; i++) { //sets new goal
            if (flicked.length() >= 3) { //reset what you flicked bc you only care about groups of 3
                flicked = "";
                break;
            }
            String goal = motif.substring(flicked.length(), flicked.length() + 1); //determines what color ball you want based on motif and what you already flicked
            for (int j = 0; j <= 2; j++) { //iterate over current inventory to check if it matches what you need in goal
                if (inventory.substring(j, j + 1).equals(goal)) {
                    flicks.add(chooseFlick(j + 1)); //add flick command to create sequential group later
                    flicked += inventory.substring(j,j+1); //remember what you flicked
                    inventory = inventory.substring(0,j) + " " + inventory.substring(j+1); //update inventory to accomodate for flicked ball
                    break;
                }
            }
        }

        for (int k = 0; k <= 2; k++) { //re-iterate over inventory to find remaining balls if any
            if (!(inventory.substring(k,k+1).equals(" "))) { // if there is a remaining ball js flick it
                flicks.add(chooseFlick(k+1)); //add the flick command
                flicked += inventory.substring(k,k+1); //update what you flicked
            }
        }
        return new SequentialGroup( //actually creates command of flicks based on what was added earlier
                flicks.get(0),

                flicks.get(1),

                flicks.get(2)
        );
    }


    public Command allDown() {
        return new ParallelGroup(
                new SetPosition(servo1, down1),
                new SetPosition(servo2, down2),
                new SetPosition(servo3, down3)
        );
    }

}