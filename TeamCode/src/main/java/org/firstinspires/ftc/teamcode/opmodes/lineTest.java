package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.baseopmode.baseAuto;
import org.firstinspires.ftc.teamcode.geometry.Vector3D;
import org.firstinspires.ftc.teamcode.stateMachine.actions.driveToPositionDifferential;

@Autonomous
public class lineTest extends baseAuto {


	@Override
	public void setStartingPosition() {
		startingPosition = new Vector3D();
	}
	@Override
	public void addActions() {
		actions.add(new driveToPositionDifferential(robot, new Vector3D(50,0,0)));
		actions.add(new driveToPositionDifferential(robot, new Vector3D(0,0,0)));

	}
}
