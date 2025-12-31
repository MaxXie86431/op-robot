package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
@TeleOp(name="SERVO TEST")
public class ServoTeleop extends OpMode {

    public static double upPosition = 0.7;
    public static double downPosition = 1;
    Servo servo;
    @Override
    public void init() {
        servo = hardwareMap.get(Servo.class, "servo");

    }

    @Override
    public void loop() {
        if(gamepad1.dpad_up)
            servo.setPosition(upPosition);
        if(gamepad1.dpad_down)
            servo.setPosition(downPosition);
        if(gamepad1.dpad_left)
            servo.setPosition(-1);
    }
}
