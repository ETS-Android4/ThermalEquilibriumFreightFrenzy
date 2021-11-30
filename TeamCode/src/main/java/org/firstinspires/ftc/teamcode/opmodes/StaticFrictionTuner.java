package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.TemplateOpModes.BaseAuto;
import org.firstinspires.ftc.teamcode.CommandBase.actions.findStaticFrictionForward;
import org.firstinspires.ftc.teamcode.CommandBase.actions.findStaticFrictionTurn;

@Autonomous
public class StaticFrictionTuner extends BaseAuto {

	@Override
	public void setStartingPosition() {

	}

	@Override
	public void addActions() {

		actions.add(new findStaticFrictionForward(robot));
		actions.add(new findStaticFrictionTurn(robot));

	}
}
