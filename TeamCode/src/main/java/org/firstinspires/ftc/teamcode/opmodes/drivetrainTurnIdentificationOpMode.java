package org.firstinspires.ftc.teamcode.opmodes;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.baseopmode.baseAuto;
import org.firstinspires.ftc.teamcode.stateMachine.actions.drivetrainTurnIdentification;

@Autonomous
public class drivetrainTurnIdentificationOpMode extends baseAuto {
	@Override
	public void setStartingPosition() {

	}

	@Override
	public void addActions() {
		actions.add(new drivetrainTurnIdentification(robot));
	}
}
