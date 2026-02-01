package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.geometry.Pose;


public class PoseStorage {
    public static Pose robotPose = new Pose(72,72,0);

    public static PoseStorage INSTANCE = new PoseStorage();

    private PoseStorage(){}
    public static void setPose(Pose pose) {
        if(pose.getHeading()<0)
            pose = pose.setHeading(2*Math.PI+pose.getHeading());


        robotPose = pose;
    }
    public static Pose getPose() {
        return robotPose;
    }
    public static double getX(){return robotPose.getX();}
    public static double getY(){return robotPose.getY();}
    public static double getHeadingDegrees() {return Math.toDegrees(robotPose.getHeading());}

    public static void resetPose() {
        robotPose = new Pose(72,72,0);
    }


}
