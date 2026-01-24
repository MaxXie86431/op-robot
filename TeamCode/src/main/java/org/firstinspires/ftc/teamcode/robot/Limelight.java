package org.firstinspires.ftc.teamcode.robot;

import static org.firstinspires.ftc.teamcode.robot.Turret.goalAngle;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.internal.hardware.android.GpioPin;
import org.firstinspires.ftc.teamcode.pedroPathing.PoseStorage;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelDeadlineGroup;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.units.Angle;
import dev.nextftc.extensions.pedro.TurnBy;

import dev.nextftc.ftc.Gamepads;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;


import java.util.List;


@Configurable
public class Limelight implements Subsystem {
    public static final Limelight INSTANCE = new Limelight();
    private Limelight() {}
    
    private Limelight3A ll;
    private double distanceFromLimelightToGoal;
    public static double goalVelocity;
    private double angleForAlignment;
    public static double slope = 295.32082;
    public static double constant = 902.26553;
    public static double padding = 55;
    public static double limelightToInchesConstant = 0.5;
    public static double angleFactor = -1.6;
    public static double llDelay = 1.25;


    @Override
    public void initialize() {
        // Use OpModeData to get hardwareMap statically (NextFTC pattern)
        ll = ActiveOpMode.hardwareMap().get(Limelight3A.class, "limelight");
        ll.start();
        ll.pipelineSwitch(1);
    }


    public double[] calculateLaunchPower() {
        LLResult result = ll.getLatestResult();
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fiducial : fiducials) {
                int id = fiducial.getFiducialId();
                if (id == 20 || id == 24) {
                    double x = fiducial.getRobotPoseTargetSpace().getPosition().x;
                    double z = fiducial.getRobotPoseTargetSpace().getPosition().z;
                    distanceFromLimelightToGoal = Math.sqrt(x * x + z * z);
                    goalVelocity = slope * distanceFromLimelightToGoal + constant + padding;
                    return new double[]{distanceFromLimelightToGoal, goalVelocity};
                }
            }
        }
        distanceFromLimelightToGoal = Math.sqrt(Math.pow((132-PoseStorage.getY()),2)+Math.pow((129-PoseStorage.getX()), 2));
        goalVelocity = slope * (distanceFromLimelightToGoal*limelightToInchesConstant) + constant + padding;
        return new double[]{distanceFromLimelightToGoal, goalVelocity};
    }

    public double calculateAlignmentAngle() {
        LLResult result = ll.getLatestResult();
        if (result != null && result.isValid()) {
            return result.getTx();
            /* 
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fiducial : fiducials) {
                int id = fiducial.getFiducialId();
                if (id == 20 || id == 24) {

                    angleForAlignment = fiducial.getTargetXDegrees();
                    return angleForAlignment;
                }
            }
                */

        }
        return 0;
    }

    public Command alignToTarget() {
        double angle = calculateAlignmentAngle();
        return new ParallelDeadlineGroup(
                new Delay(llDelay),
                new TurnBy(Angle.fromDeg(angle*angleFactor))
        );
    }

    public String color() {
        LLResult result = ll.getLatestResult();
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fiducial : fiducials) {
                int id = fiducial.getFiducialId();
                if (id == 21){
                    ActiveOpMode.telemetry().addData("Fiducial " + id, "Green, Purple, Purple");
                    ActiveOpMode.telemetry().update();
                    return "gpp";
                } else if (id == 22){
                    ActiveOpMode.telemetry().addData("Fiducial " + id, "Purple, Green, Purple");
                    ActiveOpMode.telemetry().update();
                    return "pgp";
                } else if (id == 23){
                    ActiveOpMode.telemetry().addData("Fiducial " + id, "Purple, Purple, Green");
                    ActiveOpMode.telemetry().update();
                    return "ppg";
                } else {
                    ActiveOpMode.telemetry().addData("not seeing", "null");
                    ActiveOpMode.telemetry().update();
                }
            }
        }
        return null;
    }

}
