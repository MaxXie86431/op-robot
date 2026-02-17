package org.firstinspires.ftc.teamcode.robot;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.internal.hardware.android.GpioPin;
import org.firstinspires.ftc.teamcode.pedroPathing.PoseStorage;
import org.firstinspires.ftc.teamcode.pedroPathing.Team;

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
    private static final int RED_GOAL_PIPELINE = 2;
    private static final int BLUE_GOAL_PIPELINE = 1;

    private Limelight() {}
    private Limelight3A ll;


    @Override
    public void initialize() {
        // Use OpModeData to get hardwareMap statically (NextFTC pattern)
        ll = ActiveOpMode.hardwareMap().get(Limelight3A.class, "limelight");
        ll.start();
        ll.pipelineSwitch(Team.getTeam()==0?BLUE_GOAL_PIPELINE:RED_GOAL_PIPELINE);
        ll.setPollRateHz(60);
    }

    public double calculateAlignmentAngle() {
        LLResult result = ll.getLatestResult();
        if (result != null && result.isValid()) {
            return result.getTx();
        }
        return 0;
    }

    public boolean isConnected(){
        return ll.isConnected();
    }

    public void setRedGoalPipeline() {
        ll.pipelineSwitch(RED_GOAL_PIPELINE);
    }

    public void setBlueGoalPipeline() {
        ll.pipelineSwitch(BLUE_GOAL_PIPELINE);
    }


    public void start() {
        ll.start();
    }

}
