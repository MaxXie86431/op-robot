package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Team;

@Configurable
@TeleOp(name = "DriverControlledBlue")
public class DriverControlledBlue extends DriverControlled {

    @Override
    protected void initTeam() {
        Team.setTeam(0);
    }
}