package org.firstinspires.ftc.teamcode.baseopmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.geometry.Vector3D;
import org.firstinspires.ftc.teamcode.stateMachine.actions.driveToPositionDifferential;

@Autonomous
public class blueDuckAuto extends baseAuto {
	protected Vector3D capstonePickUpSpot = new Vector3D(-36,50,Math.toRadians(-90));
	protected Vector3D placeBox = new Vector3D(-30,24,Math.toRadians(0));
	protected Vector3D spinDuck = new Vector3D(-50,50,Math.toRadians(-90));
	protected Vector3D park = new Vector3D(48,50,Math.toRadians(0));

	@Override
	public void setStartingPosition() {
		robot.setRobotPose(new Vector3D(-36,60,Math.toRadians(-90)));
	}
	@Override
	public void addActions() {
		actions.add(new driveToPositionDifferential(robot,capstonePickUpSpot)); // drive to capstone pickup

		actions.add(new driveToPositionDifferential(robot,placeBox));
		actions.add(new driveToPositionDifferential(robot,spinDuck));
		actions.add(new driveToPositionDifferential(robot,park));

	}

}
