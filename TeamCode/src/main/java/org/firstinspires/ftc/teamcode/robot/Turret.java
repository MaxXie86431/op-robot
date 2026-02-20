package org.firstinspires.ftc.teamcode.robot;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.pedroPathing.PoseStorage;
import org.firstinspires.ftc.teamcode.pedroPathing.Team;
import org.firstinspires.ftc.teamcode.utils.GenetonUtils;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.commands.utility.NullCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.hardware.controllable.RunToPosition;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;

@Configurable
public class Turret implements Subsystem {

    //PID Variable
    public static double kP = 0.00175, kI = 0, kD = 0.0001;
    //Feed Forward Values
    public static double kV = 0, kA = 0, kS = 0.05;

    public static double MANUAL_SPEED = 0.5;
    public static double TICKS_PER_DEGREE = 9.51;
    public static int RIGHT_BOUND_TICKS = 1800;
    public static int LEFT_BOUND_TICKS = -950;
    public static double PADDING = 0;


    public static boolean powerState = false;
    private static double power;
    private MotorEx turretMotor;

    public static final Turret INSTANCE = new Turret();
    public static double savedPos;

    private Turret() {}

    @Override
    public void initialize() {
        turretMotor = new MotorEx("Turret-Gear").reversed().brakeMode();
    }

    private final ControlSystem controller = ControlSystem.builder()
            .posPid(kP, kI, kD)
            .basicFF(kV, kA, kS)
            .build();

    public Command turnRight() {
        return new SequentialGroup(
                new InstantCommand(() -> {
                    powerState = false;
                }),
                new SetPower(turretMotor, MANUAL_SPEED)
        );
    }

    public Command turnLeft() {
        return new SequentialGroup(
                new InstantCommand(() -> {
                    powerState = false;
                }),
                new SetPower(turretMotor, -1*MANUAL_SPEED)
        );
    }

    public Command stop(){
        return new SequentialGroup(
            new InstantCommand(() -> powerState = false),
            new SetPower(turretMotor, 0)
        ).requires(this);
    }

    public void zero() {
        turretMotor.getMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public Command turnByDegrees(double degrees){
        return new SequentialGroup(
            new InstantCommand(() -> powerState = true),
            new RunToPosition(controller, turretMotor.getCurrentPosition()  + degrees * TICKS_PER_DEGREE)
        );
    }

    public Command turnToDegrees(double degrees){
        return new SequentialGroup(
                new InstantCommand(() -> powerState = true),
                new RunToPosition(controller, degrees * TICKS_PER_DEGREE)
        );
    }

    public Command autoAlign() {
        return turnToDegrees(GenetonUtils.INSTANCE.getTargetTurretAngle()+PADDING);
    }

    public Command tuneTurret(Gamepad gamepad){
        if(!Limelight.INSTANCE.isConnected()){
            return new NullCommand();
        }
        return new SequentialGroup(
                autoAlign(),
                tuneByLimeLight(gamepad)
        );
    }

    public Command tuneByLimeLight(Gamepad gamepad){
        double limeLightOffset = Limelight.INSTANCE.calculateAlignmentAngle();
        if(limeLightOffset!=0) {
            gamepad.rumble(500);
            GenetonUtils.LIME_LIGHT_OFFSET = limeLightOffset;
        }
        return turnToDegrees(limeLightOffset);
    }

    public Command autoAlignPerpetual = new LambdaCommand()
            .setUpdate(() -> {
                autoAlign().schedule();
            })
            .perpetually();

    public double getEncoderValue(){
        return turretMotor.getCurrentPosition();
    }

    public double getDegrees() {
        return getEncoderValue()/ TICKS_PER_DEGREE;
    }

    public void setEncoderValue(double pos) {
        turretMotor.setCurrentPosition(pos);
    }

    public Command llAlign() {
        double angle = Limelight.INSTANCE.calculateAlignmentAngle();
        if (angle != 0) {
            return turnByDegrees(angle);
        }
        return new NullCommand();
    }

    public void changePadding(double pad) {
        PADDING += pad;
    }

    @Override
    public void periodic() {
        if (powerState) {
            power = controller.calculate(turretMotor.getState());
        }

        double pos = getEncoderValue();


        if (pos >= RIGHT_BOUND_TICKS && power > 0) {
            power = 0;
        }

        if (pos <= LEFT_BOUND_TICKS && power < 0) {
            power = 0;
        }

        turretMotor.setPower(power);
    }
}
