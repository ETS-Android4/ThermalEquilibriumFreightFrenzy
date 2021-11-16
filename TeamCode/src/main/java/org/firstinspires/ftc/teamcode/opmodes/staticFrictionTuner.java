package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.baseopmode.baseAuto;
import org.firstinspires.ftc.teamcode.stateMachine.actions.findStaticFrictionForward;
import org.firstinspires.ftc.teamcode.stateMachine.actions.findStaticFrictionTurn;

@Autonomous
public class staticFrictionTuner extends baseAuto {

	@Override
	public void setStartingPosition() {

	}

	@Override
	public void addActions() {

		actions.add(new findStaticFrictionForward(robot));
		actions.add(new findStaticFrictionTurn(robot));

	}
}
