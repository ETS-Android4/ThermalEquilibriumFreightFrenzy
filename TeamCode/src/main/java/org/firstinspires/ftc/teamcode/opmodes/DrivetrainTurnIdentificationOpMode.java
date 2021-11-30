package org.firstinspires.ftc.teamcode.opmodes;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.TemplateOpModes.BaseAuto;
import org.firstinspires.ftc.teamcode.CommandBase.actions.drivetrainTurnIdentification;

@Autonomous
public class DrivetrainTurnIdentificationOpMode extends BaseAuto {
	@Override
	public void setStartingPosition() {

	}

	@Override
	public void addActions() {
		actions.add(new drivetrainTurnIdentification(robot));
	}
}
