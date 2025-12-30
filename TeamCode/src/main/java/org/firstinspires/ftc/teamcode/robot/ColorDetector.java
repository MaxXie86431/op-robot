package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

public class ColorDetector implements Subsystem {
    public static final ColorDetector INSTANCE = new ColorDetector();
    private ColorSensor colorSensor3;


    @Override
    public void initialize() {
        colorSensor3 = ActiveOpMode.hardwareMap().get(ColorSensor.class,"ColorSensor3");
    }

    public boolean isGreen(){
        return colorSensor3.green()>180;
    }

    public boolean isPurple(){
        return colorSensor3.red()>100 && colorSensor3.blue()>100;
    }


    public String getColorValues(){
        return "R: " + colorSensor3.red() + " G: " + colorSensor3.green() + " B: " + colorSensor3.blue();
    }
}
