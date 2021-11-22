package org.firstinspires.ftc.teamcode.stateMachine.actions;

import org.firstinspires.ftc.teamcode.geometry.Vector3D;
import org.firstinspires.ftc.teamcode.stateMachine.action;
import org.firstinspires.ftc.teamcode.subsystems.dashboard;
import org.firstinspires.ftc.teamcode.subsystems.robot;

public class findStaticFrictionTurn implements action {
	org.firstinspires.ftc.teamcode.subsystems.robot robot;
	double power = 0;
	double iteration = 0.0005;
	boolean isComplete = false;
	Vector3D initialPosition;

	public findStaticFrictionTurn(robot robot) {
		this.robot = robot;
	}

	@Override
	public void startAction() {
		initialPosition = robot.getRobotPose();
	}

	@Override
	public void runAction() {

		robot.driveTrain.setMotorPowers(power, -power);
		power += iteration;
		if (robot.getRobotPose().angle.getRadians() > Math.abs(0.17)) {
			isComplete = true;
		}
		dashboard.packet.put("turn power", power);


	}

	@Override
	public void stopAction() {
		robot.driveTrain.STOP();
	}

	@Override
	public boolean isActionComplete() {
		if (isComplete) {
			System.out.println("minimum motor power turn is " + power);
		}
		return isComplete;
	}


	@Override
	public boolean isActionPersistent() {
		return false;
	}

}
