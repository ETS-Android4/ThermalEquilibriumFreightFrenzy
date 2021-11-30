package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.TemplateOpModes.BaseAuto;
import org.firstinspires.ftc.teamcode.Geometry.Vector3D;
import org.firstinspires.ftc.teamcode.CommandBase.actions.driveToPositionDifferential;

@Autonomous
public class LineTest extends BaseAuto {


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
