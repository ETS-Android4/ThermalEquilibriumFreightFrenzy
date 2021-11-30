package org.firstinspires.ftc.teamcode.commandBase.autoActions;

import org.firstinspires.ftc.teamcode.geometry.Vector3D;
import org.firstinspires.ftc.teamcode.commandBase.action;
import org.firstinspires.ftc.teamcode.subsystems.Dashboard;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

public class FindStaticFrictionForward implements action {

	Robot robot;
	double power = 0;
	double iteration = 0.0003;
	boolean isComplete = false;
	Vector3D initialPosition;
	double validDistance = 0.025;

	public FindStaticFrictionForward(Robot robot) {
		this.robot = robot;
	}

	@Override
	public void startAction() {
		initialPosition = robot.getRobotPose();
	}

	@Override
	public void runAction() {

		robot.driveTrain.setMotorPowers(power, power);
		power += iteration;
		if (robot.getRobotPose().distanceToPose(initialPosition) > validDistance) {
			isComplete = true;
		}
		Dashboard.packet.put("drive power", power);


	}

	@Override
	public void stopAction() {
		robot.driveTrain.STOP();
	}

	@Override
	public boolean isActionComplete() {
		if (isComplete) {
			System.out.println("minimum motor power forward is " + power);
		}
		return isComplete;
	}

	@Override
	public boolean isActionPersistent() {
		return false;
	}
}
