package org.firstinspires.ftc.teamcode.opmodes.competetionOpModes;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.geometry.Vector3D;


@Autonomous
public class BlueDuckAuto extends RedDuckAuto {
	@Override
	public void setStartingPosition() {
		startPosition = new Vector3D(-39, 56, Math.toRadians(90));
		goalPosition = new Vector3D(-12, 24, 0);
		carousel = new Vector3D(-72,60,0);
		park = new Vector3D(-60,35,0);
		robot.setRobotPose(startPosition);
		leftCapStone = new Vector3D(-48 - 20, 36,0 );
		middleCapstone = new Vector3D(-48 - 12, 36,0);
		rightCapstone = new Vector3D(-48 - 2, 36,0);

	}

	@Override
	public void setVisionSettings() {
		setVisionForRightVisible();
	}
}
