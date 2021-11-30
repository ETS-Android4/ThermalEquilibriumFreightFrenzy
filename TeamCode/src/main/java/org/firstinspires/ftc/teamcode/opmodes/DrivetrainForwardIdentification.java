package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.TemplateOpModes.BaseAuto;
import org.firstinspires.ftc.teamcode.CommandBase.actions.drivetrainIdentificationForward;

@Autonomous
public class DrivetrainForwardIdentification extends BaseAuto {
	@Override
	public void setStartingPosition() {

	}

	@Override
	public void addActions() {
		actions.add(new drivetrainIdentificationForward(robot));
	}
}
