package org.firstinspires.ftc.teamcode.opmodes.tuningOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.templateOpModes.BaseAuto;
import org.firstinspires.ftc.teamcode.commandBase.autoActions.DrivetrainIdentificationForward;

@Autonomous
public class DrivetrainForwardIdentification extends BaseAuto {
	@Override
	public void setStartingPosition() {

	}

	@Override
	public void addActions() {
		actions.add(new DrivetrainIdentificationForward(robot));
	}
}
