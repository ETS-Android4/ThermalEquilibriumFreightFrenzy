package org.firstinspires.ftc.teamcode.stateMachine.actions;

import org.firstinspires.ftc.teamcode.basedControl.basedPID;
import org.firstinspires.ftc.teamcode.classicalControl.PIDFCoefficients;
import org.firstinspires.ftc.teamcode.stateMachine.action;
import org.firstinspires.ftc.teamcode.subsystems.robot;

public class basedTurn implements action {

	robot robot;
	basedPID pid;
	PIDFCoefficients coefficients = new PIDFCoefficients(1.55,0.01,0.12);
	double targetAngle;
	boolean isComplete = false;

	public basedTurn(robot robot, double targetAngle) {
		this.robot = robot;
		this.targetAngle = targetAngle;
	}

	@Override
	public void startAction() {
		pid = new basedPID(coefficients,targetAngle,3,0.004,Math.toRadians(1));
	}

	@Override
	public void runAction() {
		double output = pid.calculateAngle(robot.odometry.subsystemState().getAngleRadians());
		robot.driveTrain.robotRelative(0,output);
		isComplete = pid.isComplete();
	}

	@Override
	public void stopAction() {
		robot.driveTrain.STOP();
	}

	@Override
	public boolean isActionComplete() {
		return isComplete;
	}

	@Override
	public boolean isActionPersistent() {
		return true;
	}
}
