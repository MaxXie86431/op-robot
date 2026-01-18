package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.geometry.Pose;


public class PoseStorage {
    public static Pose robotPose = new Pose(0,0,0);
    public static void setPose(Pose pose) {
        robotPose = pose;
    }
    public static Pose getPose() {
        return robotPose;
    }
}
