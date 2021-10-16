package org.firstinspires.ftc.teamcode.stateMachine.actions;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.basedControl.basedControl;
import org.firstinspires.ftc.teamcode.geometry.Vector3D;
import org.firstinspires.ftc.teamcode.stateMachine.action;
import org.firstinspires.ftc.teamcode.subsystems.dashboard;
import org.firstinspires.ftc.teamcode.subsystems.robot;

import static org.firstinspires.ftc.teamcode.basedControl.robotCoefficients.driveCoefficients;
import static org.firstinspires.ftc.teamcode.basedControl.robotCoefficients.turnCoefficients;

public class basedDrive implements action {

	protected double targetDistance;
	protected double distance;
	protected boolean isComplete = false;

	protected robot robot;
	protected Vector3D initialPosition;

	protected basedControl turnPid;
	protected basedControl drivePid;


	public basedDrive(robot robot, double targetDistance) {
		this.robot = robot;
		this.targetDistance = targetDistance;
	}

	@Override
	public void startAction() {
		initialPosition = robot.odometry.subsystemState();
		turnPid = new basedControl(turnCoefficients, initialPosition.getAngleRadians(), 3, 0.004, Math.toRadians(1));
		drivePid = new basedControl(driveCoefficients, Math.abs(targetDistance), 3, 0.3, 1);
	}

	@Override
	public void runAction() {

		distance = initialPosition.distanceToPose(robot.odometry.subsystemState());
		dashboard.packet.put("distance traveled", distance);

		double drive = Range.clip(Math.signum(targetDistance) * drivePid.calculate(distance), -1, 1);
		double turn = turnPid.calculateLinearAngle(robot.odometry.subsystemState().getAngleRadians());
		turn = Range.clip(turn, -1, 1);

		robot.driveTrain.robotRelative(drive, turn);

		isComplete = drivePid.isComplete();

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
