package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ColorDetector {
    private ColorSensor colorSensor3 = null;
    private HardwareMap hwMap;

    public ColorDetector(HardwareMap hardwareMap){
        hwMap = hardwareMap;
    }

    public void init(){
        colorSensor3 = hwMap.get(ColorSensor.class, "ColorSensor3");
    }

    public boolean isGreen(){
        return colorSensor3.green()>180;
    }

    public String getColorValues(){
        return "R: " + colorSensor3.red() + " G: " + colorSensor3.green() + " B: " + colorSensor3.blue();
    }
}
