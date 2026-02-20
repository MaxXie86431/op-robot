package org.firstinspires.ftc.teamcode.robot;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.ColorSensor;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

@Configurable
public class ColorDetector implements Subsystem {
    public static final ColorDetector INSTANCE = new ColorDetector();
    private ColorDetector() {}
    private ColorSensor colorSensorfirst1, colorSensorfirst2, colorSensorsecond1, colorSensorsecond2, colorSensorthird1, colorSensorthird2;
    public static int green2 = 180;
    public static int green3 = 1000;
    public static int red2 = 100;
    public static int blue2 = 100;
    public static int red3 = 1000;
    public static int blue3 = 1000;



    @Override
    public void initialize() {
        colorSensorfirst1 = ActiveOpMode.hardwareMap().get(ColorSensor.class, "ColorSensorfirst1");
        colorSensorfirst2 = ActiveOpMode.hardwareMap().get(ColorSensor.class, "ColorSensorfirst2");
        colorSensorsecond1 = ActiveOpMode.hardwareMap().get(ColorSensor.class, "ColorSensorsecond1");
        colorSensorsecond2 = ActiveOpMode.hardwareMap().get(ColorSensor.class, "ColorSensorsecond2");
        colorSensorthird1 = ActiveOpMode.hardwareMap().get(ColorSensor.class, "ColorSensorthird1");
        colorSensorthird2 = ActiveOpMode.hardwareMap().get(ColorSensor.class, "ColorSensorthird2");

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



    public String getSensorTelemetry(ColorSensor sensor, int v){


        if (v==3) {
            return "R: " + sensor.red() + " G: " + Math.round(sensor.green()*0.6) + " B: " + sensor.blue();
        }
        return "R: " + sensor.red() + " G: " + sensor.green() + " B: " + sensor.blue();
    }

    public double getGreen(int pos) {
        if (pos==1) {
            return colorSensorfirst1.green();
        }
        else if (pos==2) {
            return colorSensorfirst2.green();
        }
        else if (pos==3) {
            return colorSensorsecond1.green();
        }
        else if (pos==4) {
            return colorSensorsecond2.green();
        }
        else if (pos==5) {
            return Math.round(colorSensorthird1.green()*0.6);
        }
        else if (pos==6) {
            return colorSensorthird2.green();
        }
        else {
            return 0;
        }
    }

    public int[] getColors() {
        double max = 0;
        int pos = 0;
        for (int i = 1; i < 7; i++) {
            if (getGreen(i) > max) {
                max = getGreen(i);
                pos = i;
            }
        }

        if (pos == 1 || pos == 2) {
            return new int[]{1,0,0};
        }
        else if (pos==3||pos==4) {
            return new int[]{0,1,0};
        }
        else if (pos==5||pos==6) {
            return new int[]{0,0,1};
        }

        return new int[]{0,0,0};

    }



    public String getSensorValues() {
        //return getColor(colorSensor1) + getColor(colorSensor2) + getColor(colorSensor3);
        return "\n" + getSensorTelemetry(colorSensorfirst1, 2) + "\n" + getSensorTelemetry(colorSensorfirst2, 2) + "\n" + getSensorTelemetry(colorSensorsecond1,2) + "\n" + getSensorTelemetry(colorSensorsecond2,2) + "\n" + getSensorTelemetry(colorSensorthird1,3) + "\n" + getSensorTelemetry(colorSensorthird2,2);
    }

    public Boolean hasOpenSlots() {
        return getSensorValues().contains(" ");
    }
}