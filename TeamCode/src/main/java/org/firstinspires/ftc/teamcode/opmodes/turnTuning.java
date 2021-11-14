package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.baseopmode.baseAuto;
import org.firstinspires.ftc.teamcode.stateMachine.actions.basedTurn;

@Autonomous
public class turnTuning extends baseAuto {
	@Override
	public void addActions() {
		actions.add(new basedTurn(robot, Math.toRadians(90)));
	}
}
