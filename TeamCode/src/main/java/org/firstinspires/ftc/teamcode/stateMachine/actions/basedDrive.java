package org.firstinspires.ftc.teamcode.stateMachine.actions;

import org.firstinspires.ftc.teamcode.basedControl.basedControl;
import org.firstinspires.ftc.teamcode.basedControl.controllerCoefficients;
import org.firstinspires.ftc.teamcode.classicalControl.PIDFCoefficients;
import org.firstinspires.ftc.teamcode.filter.LowPassFilter;
import org.firstinspires.ftc.teamcode.geometry.Vector3D;
import org.firstinspires.ftc.teamcode.stateMachine.action;
import org.firstinspires.ftc.teamcode.subsystems.dashboard;
import org.firstinspires.ftc.teamcode.subsystems.robot;

import java.util.Vector;

import static org.firstinspires.ftc.teamcode.subsystems.robot.isCompBot;

public class basedDrive implements action {

	protected double targetDistance;
	protected double distance;
	protected boolean isComplete = false;

	protected robot robot;
	protected Vector3D initialPosition;

	protected basedControl turnPid;
	protected basedControl drivePid;
	protected PIDFCoefficients turnCoefficients;
	protected PIDFCoefficients driveCoefficients;
	protected Vector3D targetPosition = null;

	protected LowPassFilter profile = new LowPassFilter(0.8);

	public basedDrive(robot robot, double targetDistance) {
		this.robot = robot;
		this.targetDistance = targetDistance;
		if (isCompBot) {
			turnCoefficients = controllerCoefficients.compBotDriveCorrect;
			driveCoefficients = controllerCoefficients.compBotDrive;
		} else {
			turnCoefficients = controllerCoefficients.protoBotDriveCorrect;
			driveCoefficients = controllerCoefficients.protoBotDrive;
		}
	}

	public basedDrive(robot robot, Vector3D targetPosition, double scaler) {
		this.robot = robot;
		if (isCompBot) {
			turnCoefficients = controllerCoefficients.compBotDriveCorrect;
			driveCoefficients = controllerCoefficients.compBotDrive;
		} else {
			turnCoefficients = controllerCoefficients.protoBotDriveCorrect;
			driveCoefficients = controllerCoefficients.protoBotDrive;
		}
		this.targetDistance = scaler;
		this.targetPosition = targetPosition;
	}

	@Override
	public void startAction() {
		initialPosition = robot.odometry.subsystemState();
		turnPid = new basedControl(turnCoefficients, initialPosition.getAngleRadians(), 3, 0.004, Math.toRadians(1));
		drivePid = new basedControl(driveCoefficients, Math.abs(targetDistance), 3, 0.3, 1);
		if (targetPosition != null) {

			Vector3D positionError = targetPosition.getError(initialPosition);
			double omega1 = Math.atan2(positionError.getY(),positionError.getX());
			double omega2 = Math.atan2(positionError.getY(),positionError.getX()) - Math.toRadians(180);

			this.targetDistance = Math.abs(this.targetPosition.distanceToPose(initialPosition)) * this.targetDistance;
			drivePid.setReference(Math.abs(this.targetDistance));

			if (this.targetDistance > 0) {
				turnPid.setReference(omega1);
			}
			else {
				turnPid.setReference(omega2);
			}
		}
	}

	@Override
	public void runAction() {

		distance = initialPosition.distanceToPose(robot.odometry.subsystemState());
		dashboard.packet.put("distance traveled", distance);
		double targetDistProfile = profile.updateEstimate(Math.abs(targetDistance));
		dashboard.packet.put("target ", targetDistProfile);
		drivePid.setReference(targetDistProfile);
		double drive = drivePid.calculate(distance) * Math.signum(targetDistance);
		double turn = turnPid.calculateLinearAngle(robot.odometry.subsystemState().getAngleRadians());

		robot.driveTrain.robotRelative(drive, turn);

		isComplete = drivePid.isComplete() || drivePid.isVeryStable();

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
