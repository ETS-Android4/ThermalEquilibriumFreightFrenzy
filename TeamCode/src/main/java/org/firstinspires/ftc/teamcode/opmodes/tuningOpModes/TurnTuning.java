package org.firstinspires.ftc.teamcode.opmodes.tuningOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.templateOpModes.BaseAuto;
import org.firstinspires.ftc.teamcode.commandBase.autoActions.Turn;

@Autonomous
public class TurnTuning extends BaseAuto {
	@Override
	public void setStartingPosition() {

	}

	@Override
	public void addActions() {
		actions.add(new Turn(robot, Math.toRadians(90)));
	}
}
