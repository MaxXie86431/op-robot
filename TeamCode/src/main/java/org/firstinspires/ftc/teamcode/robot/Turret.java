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
    //-1000 1700
    public static double kP = 0.005;
    public static double kI = 0;
    public static double kD = 0;
    public static double kV = 0.025;
    public static double kA = 0.02;
    public static double kS = 0.03;
    public static double velocity = 0.5;
    public static double ticksPerRev = 753.2;
    public static double rotationRatio = 0.22;
    public static double positionPerDegree = 9.51;
    public static boolean powerState = false;
    public static double goalAngle;
    public static double heading;
    public static double angle;
    public static int tolerance= 50;
    public static int rightBound = 1800;
    public static int leftBound = -950;
    public static boolean locked = false;
    private static double power;
    public static double blueGoalX=12;
    public static double blueGoalY=132;

    private final ControlSystem controller = ControlSystem.builder()
            .posPid(kP, kI, kD)
            //.basicFF()
            .basicFF(kV, kA, kS)
            .build();

    public static final Turret INSTANCE = new Turret();

    private MotorEx turretMotor;

    private Turret() {}

    @Override
    public void initialize() {
        turretMotor = new MotorEx("Turret-Gear").reversed().brakeMode();
    }

    public Command turnRight() {
        return new SequentialGroup(
                new InstantCommand(() -> powerState = false),
                new SetPower(turretMotor, velocity)
        );
    }

    public Command turnLeft(){
        return new SequentialGroup(
            new InstantCommand(() -> powerState = false),
            new SetPower(turretMotor, -1 * velocity)
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

    public Command goToZero(){
        return new SequentialGroup(
                new InstantCommand(() -> powerState = true),
                new RunToPosition(controller, 0)
        ).requires(this);
    }

    public Command turnByDegrees(double degrees){
        return new SequentialGroup(
            new InstantCommand(() -> powerState = true),
            new RunToPosition(controller, turretMotor.getCurrentPosition()  + degrees * positionPerDegree)
        );
    }

    public Command turnToDegrees(double degrees){
        return new SequentialGroup(
                new InstantCommand(() -> powerState = true),
                new RunToPosition(controller, degrees * positionPerDegree)
        );
    }

    public double getMoveAngle(){
        //double turretPos = -1 * getDegrees();
        //double angle = (heading-tagPos) + (turretPos-90);
        heading = PoseStorage.getHeadingDegrees();
        if (Team.getTeam() == 0) {
            //goalAngle = (180-(Math.toDegrees(Math.atan2(132 - PoseStorage.getY(), PoseStorage.getX()-15))));
            goalAngle = (180-(Math.toDegrees(Math.atan2(blueGoalY - PoseStorage.getY(), PoseStorage.getX()-blueGoalX))));
            angle = heading+getDegrees()-goalAngle;
            return angle;
            //return turnByDegrees((heading>goalAngle?-angle:angle));
        }
        else {
            //goalAngle = Math.toDegrees(Math.atan2(132 - PoseStorage.getY(), 129 - PoseStorage.getX()));
            goalAngle = Math.toDegrees(Math.atan2(132 - PoseStorage.getY(), 126 - PoseStorage.getX()));
            angle = goalAngle-heading+getDegrees();
            if (angle >= 0) {
                if (angle >= 180) {
                    angle = 360 - angle;
                }
            }
            else if (angle < 0) {
                if (angle <= -180) {
                    angle = angle + 360;
                }
            }
            return -1 * angle;
        }
    }

    public Command autoAlignTrig() {
        angle = getMoveAngle();
        return turnByDegrees(angle);
    }


    public Command autoAlignPerpetual = new LambdaCommand()
            .setUpdate(() -> {
                autoAlignTrig().schedule();
            })
            .perpetually();


    public double getEncoderValue(){
        return turretMotor.getCurrentPosition();
    }
    public double getDegrees() {
        return getEncoderValue()/positionPerDegree;
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
        /*
        if (getEncoderValue() > leftBound && getEncoderValue() < rightBound) {
            if (powerState) {
                turretMotor.setPower(controller.calculate(turretMotor.getState()));
            }
        }
        else {
                turretMotor.setPower(0);
                locked = true;
        }
        */
        if (powerState) {
            power = controller.calculate(turretMotor.getState());
        }

        double pos = getEncoderValue();


        if (pos >= rightBound && power > 0) {
            power = 0;
        }


        if (pos <= leftBound && power < 0) {
            power = 0;
        }

        turretMotor.setPower(power);



        /*
        directions mean actually turning the turret (eg right means turning turret all the way right)
        - right bound: 1900 encoder, 200 degrees of turret
        - left bound: -1050 encoder, -110.75 degrees of turret
         */

    }
}
