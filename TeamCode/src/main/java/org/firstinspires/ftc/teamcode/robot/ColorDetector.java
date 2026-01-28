package org.firstinspires.ftc.teamcode.robot;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.ColorSensor;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

@Configurable
public class ColorDetector implements Subsystem {
    public static final ColorDetector INSTANCE = new ColorDetector();
    private ColorDetector() {}
    private ColorSensor colorSensor1;
    private ColorSensor colorSensor2;
    private ColorSensor colorSensor3;
    public static int green2 = 180;
    public static int green3 = 1000;
    public static int red2 = 100;
    public static int blue2 = 100;
    public static int red3 = 1000;
    public static int blue3 = 1000;



    @Override
    public void initialize() {
        colorSensor1 = ActiveOpMode.hardwareMap().get(ColorSensor.class,"ColorSensor1");
        colorSensor2 = ActiveOpMode.hardwareMap().get(ColorSensor.class,"ColorSensor2");
        colorSensor3 = ActiveOpMode.hardwareMap().get(ColorSensor.class,"ColorSensor3");
    }

    public boolean isGreen(ColorSensor sensor, int num){
        if (num == 2) {
            return sensor.green()>green2;
        }
        else{
            return sensor.green() > green3;
        }

    }

    public boolean isPurple(ColorSensor sensor, int num) {
        if (num == 2) {
            return sensor.red()>red2 && sensor.blue()>blue2;
        }
        else {
            return sensor.red() > red3 && sensor.blue() > blue3;
        }

    }



    public String getSensorTelemetry(ColorSensor sensor){
        return "R: " + sensor.red() + " G: " + sensor.green() + " B: " + sensor.blue();
    }

    public String getColor(ColorSensor sensor, int num) {
        if (isGreen(sensor, num)) {
            return "g";
        }
        else if (isPurple(sensor, num)) {
            return "p";
        }
        else {
            return " ";
        }
    }

    public String chooseGetColor(int pos) {
        if (pos == 1) {
            return getColor(colorSensor1, 2);
        }
        else if (pos == 2) {
            return getColor(colorSensor2, 3);
        }
        else if (pos == 3){
            return getColor(colorSensor3, 2);
        }
        else {
            return null;
        }
    }

    public String getSensorValues() {
        //return getColor(colorSensor1) + getColor(colorSensor2) + getColor(colorSensor3);
        return "\n" + getSensorTelemetry(colorSensor1) + "\n" + getSensorTelemetry(colorSensor2) + "\n" + getSensorTelemetry(colorSensor3);
    }

    public Boolean hasOpenSlots() {
        return getSensorValues().contains(" ");
    }
}