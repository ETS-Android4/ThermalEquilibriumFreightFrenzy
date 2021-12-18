package org.firstinspires.ftc.teamcode.opmodes.competetionOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.geometry.Vector3D;

@Autonomous
public class BlueWareHouse extends RedWareHouse {

	@Override
	public void setStartingPosition() {
		startPosition = new Vector3D(9, 56, Math.toRadians(90));
		goal = new Vector3D(-12, 20, 0);
		robot.setRobotPose(startPosition);
	}

	@Override
	public void setVisionSettings() {
		setVisionForRightVisible();
	}

}
