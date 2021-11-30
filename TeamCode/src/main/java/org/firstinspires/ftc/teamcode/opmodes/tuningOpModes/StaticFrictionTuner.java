package org.firstinspires.ftc.teamcode.opmodes.tuningOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.templateOpModes.BaseAuto;
import org.firstinspires.ftc.teamcode.commandBase.autoActions.FindStaticFrictionForward;
import org.firstinspires.ftc.teamcode.commandBase.autoActions.FindStaticFrictionTurn;

@Autonomous
public class StaticFrictionTuner extends BaseAuto {

	@Override
	public void setStartingPosition() {

	}

	@Override
	public void addActions() {

		actions.add(new FindStaticFrictionForward(robot));
		actions.add(new FindStaticFrictionTurn(robot));

	}
}
