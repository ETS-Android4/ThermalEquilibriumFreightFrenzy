package org.firstinspires.ftc.teamcode.opmodes.testOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commandBase.autoActions.DrivetrainControl.Drive;
import org.firstinspires.ftc.teamcode.commandBase.autoActions.DrivetrainControl.Turn;
import org.firstinspires.ftc.teamcode.commandBase.autoActions.Misc.Delay;
import org.firstinspires.ftc.teamcode.geometry.Vector3D;
import org.firstinspires.ftc.teamcode.templateOpModes.BaseAuto;

@Autonomous
public class PathFollowingTest extends BaseAuto {

	boolean square = false;
	@Override
	public void setStartingPosition() {

	}

	@Override
	public void setVisionSettings() {

	}

	@Override
	public void addActions() {

		if (square) {
			actions.add(new Drive(robot, new Vector3D(30,0,0),1,0));
			actions.add(new Drive(robot, new Vector3D(30,30,0),-1,0));
			actions.add(new Drive(robot, new Vector3D(0,30,0),1,0));
			actions.add(new Drive(robot, new Vector3D(0,0,0),-1,0));
			actions.add(new Turn(robot,0));
			return;
		}

//		actions.add(new Drive(robot, new Vector3D(30,30,0),1,0));
//		actions.add(new Drive(robot, new Vector3D(60,40,0),-1,0));
//		actions.add(new Drive(robot, new Vector3D(100,0,0),-1,0));
//
//		actions.add(new Drive(robot, new Vector3D(0,0,0),-1,0));
//		actions.add(new Turn(robot,0));

		for (int i = 0; i < 3; ++i) {
			actions.add(new Drive(robot, new Vector3D(36 + (i * 24),30,0),1,0));
			actions.add(new Drive(robot, new Vector3D(36 + (i * 24),-20,0),1,0));
			actions.add(new Drive(robot, new Vector3D(36 + (i * 24),30,0),-1,0));
		}
		actions.add(new Drive(robot, new Vector3D(0,0,0),-1,0));
		actions.add(new Turn(robot,0));



	}
}
