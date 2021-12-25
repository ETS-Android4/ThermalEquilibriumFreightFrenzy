package org.firstinspires.ftc.teamcode.opmodes.testOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commandBase.autoActions.DrivetrainControl.Drive;
import org.firstinspires.ftc.teamcode.commandBase.autoActions.DrivetrainControl.Turn;
import org.firstinspires.ftc.teamcode.commandBase.autoActions.Misc.Delay;
import org.firstinspires.ftc.teamcode.geometry.Vector3D;
import org.firstinspires.ftc.teamcode.templateOpModes.BaseAuto;

@Autonomous
public class PathFollowingTest extends BaseAuto {
	@Override
	public void setStartingPosition() {

	}

	@Override
	public void setVisionSettings() {

	}

	@Override
	public void addActions() {
		actions.add(new Drive(robot, new Vector3D(30,0,0),1,0));
		actions.add(new Drive(robot, new Vector3D(30,30,0),-1,0));
		actions.add(new Drive(robot, new Vector3D(0,30,0),1,0));
		actions.add(new Drive(robot, new Vector3D(0,0,0),-1,0));
		actions.add(new Turn(robot,0));

	}
}
