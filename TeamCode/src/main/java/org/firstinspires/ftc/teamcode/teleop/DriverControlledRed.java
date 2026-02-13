package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Team;

@Configurable
@TeleOp(name = "DriverControlledRed")
public class DriverControlledRed extends DriverControlled {

    @Override
    protected void initTeam() {
        Team.setTeam(1);
    }
}