package org.firstinspires.ftc.teamcode.utils;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.pedroPathing.PoseStorage;
import org.firstinspires.ftc.teamcode.pedroPathing.Team;
import org.firstinspires.ftc.teamcode.robot.Limelight;

@Configurable
public class GenetonUtils {

    private static final double TARGET_X_BLUE = 12.0, TARGET_X_RED =126.0, TARGET_Y = 132;

    public static double SLOPE = 5.19679, CONSTANT = 773.09206,  PADDING=20;

    public static double LIME_LIGHT_OFFSET = 0.0;

    public static boolean USE_LIMELIGHT = true;

    public static final GenetonUtils INSTANCE = new GenetonUtils();

    private GenetonUtils(){}


    public double getTargetTurretAngle() {
        if(USE_LIMELIGHT){
            LIME_LIGHT_OFFSET = Limelight.INSTANCE.calculateAlignmentAngle();
        }
        return getTargetAngle() + LIME_LIGHT_OFFSET;
    }

    public double getLimeLightTargetAngle(double limelightOffset){
        double robotHeading = PoseStorage.getHeadingDegrees();
        return normalizeAngle(robotHeading + limelightOffset);
    }

    public double getTargetAngle(){
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

    public double getTargetLength(){
        double targetX = (Team.getTeam() == 0) ? TARGET_X_BLUE : TARGET_X_RED;
        double deltaX = targetX - PoseStorage.getX();
        double deltaY = TARGET_Y - PoseStorage.getY();
        return Math.sqrt(deltaX*deltaX + deltaY*deltaY);
    }

    public double getTargetVelocity(){
        double d = getTargetLength();
        return SLOPE * d + CONSTANT + PADDING;
    }

    private double normalizeAngle(double angle) {
        while (angle > 180)  angle -= 360;
        while (angle <= -180) angle += 360;
        return angle;
    }
    public void changePadding(double pad) {
        PADDING += pad;
    }
}
