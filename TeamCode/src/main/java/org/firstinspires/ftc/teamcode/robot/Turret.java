package org.firstinspires.ftc.teamcode.robot;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.pedroPathing.PoseStorage;
import org.firstinspires.ftc.teamcode.pedroPathing.Team;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.commands.utility.NullCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.RunToPosition;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;

@Configurable
public class Turret implements Subsystem {

    //PID Variable
    public static double kP = 0.005, kI = 0, kD = 0;
    //Feed Forward Values
    public static double kV = 0.025, kA = 0.02, kS = 0.03;

    public static double MANUAL_SPEED = 0.5;
    public static double TICKS_PER_DEGREE = 9.51;
    public static int RIGHT_BOUND_TICKS = 1800;
    public static int LEFT_BOUND_TICKS = -950;

    private static final double TARGET_X_BLUE = 12.0, TARGET_X_RED =126.0, TARGET_Y = 132;

    public static boolean locked = false;
    public static boolean powerState = false;
    private static double power;
    private MotorEx turretMotor;

    public static final Turret INSTANCE = new Turret();

    private Turret() {}

    @Override
    public void initialize() {
        turretMotor = new MotorEx("Turret-Gear").reversed().zeroed().brakeMode();
    }

    private final ControlSystem controller = ControlSystem.builder()
            .posPid(kP, kI, kD)
            .basicFF(kV, kA, kS)
            .build();

    public Command turnRight() {
        return new SequentialGroup(
                new InstantCommand(() -> powerState = false),
                new SetPower(turretMotor, MANUAL_SPEED)
        );
    }

    public Command turnLeft(){
        return new SequentialGroup(
            new InstantCommand(() -> powerState = false),
            new SetPower(turretMotor, -1 * MANUAL_SPEED)
        );
    }

    public Command stop(){
        return new SequentialGroup(
            new InstantCommand(() -> powerState = false),
            new SetPower(turretMotor, 0)
        ).requires(this);
    }

    public void zero() {
        turretMotor.zero();
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


    public double getTargetTurretAngle() {
        double robotHeading = PoseStorage.getHeadingDegrees();
        double fieldAngleToTarget = getFieldTargetAngle();
        return normalizeAngle(robotHeading - fieldAngleToTarget);
    }

    public double getFieldTargetAngle(){
        double targetX = (Team.getTeam() == 0) ? TARGET_X_BLUE : TARGET_X_RED;
        double deltaX = targetX - PoseStorage.getX();
        double deltaY = TARGET_Y - PoseStorage.getY();
        return Math.toDegrees(Math.atan2(deltaY, deltaX));
    }

    public Command autoAlign() {
        return turnToDegrees(getTargetTurretAngle());
    }

    private double normalizeAngle(double angle) {
        while (angle > 180)  angle -= 360;
        while (angle <= -180) angle += 360;
        return angle;
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

    public void setEncoderValue(int pos) {
        turretMotor.setCurrentPosition(pos);
    }

    public Command autoTrackButton() {
        double angle = Limelight.INSTANCE.calculateAlignmentAngle();
        if (angle != 0) {
            return turnByDegrees(angle);
        }
        return new NullCommand();
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
