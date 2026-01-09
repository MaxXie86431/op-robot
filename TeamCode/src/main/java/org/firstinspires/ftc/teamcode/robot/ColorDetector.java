package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.ColorSensor;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

public class ColorDetector implements Subsystem {
    public static final ColorDetector INSTANCE = new ColorDetector();
    private ColorDetector() {}
    private ColorSensor colorSensor1;
    private ColorSensor colorSensor2;
    private ColorSensor colorSensor3;


    @Override
    public void initialize() {
        colorSensor1 = ActiveOpMode.hardwareMap().get(ColorSensor.class,"ColorSensor1");
        colorSensor2 = ActiveOpMode.hardwareMap().get(ColorSensor.class,"ColorSensor2");
        colorSensor3 = ActiveOpMode.hardwareMap().get(ColorSensor.class,"ColorSensor3");
    }

    public boolean isGreen(ColorSensor sensor){
        return sensor.green()>180;
    }

    public boolean isPurple(ColorSensor sensor) {
        return sensor.red()>100 && sensor.blue()>100;
    }

    public boolean isPurple(){
        return colorSensor3.red()>100 && colorSensor3.blue()>100;
    }


    public String getSensorTelemetry(ColorSensor sensor){
        return "R: " + sensor.red() + " G: " + sensor.green() + " B: " + sensor.blue();
    }

    public String getColor(ColorSensor sensor) {
        if (isGreen(sensor)) {
            return "g";
        }
        else if (isPurple(sensor)) {
            return "p";
        }
        else {
            return " ";
        }
    }

    public String chooseGetColor(int pos) {
        if (pos == 1) {
            return getColor(colorSensor1);
        }
        else if (pos == 2) {
            return getColor(colorSensor2);
        }
        else if (pos == 3){
            return getColor(colorSensor3);
        }
        else {
            return null;
        }
    }

    public String getSensorValues() {
        return getColor(colorSensor1) + getColor(colorSensor2) + getColor(colorSensor3);
    }

    public Boolean checkSlotsCapacity() {
        return getSensorValues().indexOf(" ") >= 0;
    }
}
