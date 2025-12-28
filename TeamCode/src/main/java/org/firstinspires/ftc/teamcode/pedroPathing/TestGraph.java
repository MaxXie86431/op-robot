package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Test Graph")
public class TestGraph extends OpMode {
    //private final GraphManager graphManager = PanelsGraph.INSTANCE.getManager();
    private final TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

    private final ElapsedTime timer = new ElapsedTime();

    private double sinVariable = 0.0;
    private double cosVariable = 0.0;
    private double constVariable = 0.0;

    @Override
    public void init() {
        timer.reset();
        updateSignals();
    }

    @Override
    public void loop() {
        updateSignals();
    }

    private void updateSignals() {
        double t = timer.seconds();
        sinVariable = Math.sin(t);
        cosVariable = Math.cos(t);

        panelsTelemetry.addData("sin", sinVariable);
        panelsTelemetry.addData("cos", cosVariable);
        panelsTelemetry.addData("const", constVariable);
        panelsTelemetry.update();

        panelsTelemetry.addData("sin", sinVariable);
        panelsTelemetry.addData("cos", cosVariable);
        panelsTelemetry.addData("const", constVariable);
        panelsTelemetry.update(telemetry);
    }
}

