package org.firstinspires.ftc.teamcode.CommandBase.actions;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.controls.RobustPID;
import org.firstinspires.ftc.teamcode.controls.controllerCoefficients;
import org.firstinspires.ftc.teamcode.classicalControl.PIDFCoefficients;
import org.firstinspires.ftc.teamcode.CommandBase.action;
import org.firstinspires.ftc.teamcode.subsystems.Dashboard;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

import static org.firstinspires.ftc.teamcode.subsystems.Robot.isCompBot;

public class basedTurn implements action {

	Robot robot;
	RobustPID pid;

	PIDFCoefficients coefficients;
	double targetAngle;
	boolean isComplete = false;
	ElapsedTime timer = new ElapsedTime();

	public basedTurn(Robot robot, double targetAngle) {
		this.robot = robot;
		this.targetAngle = targetAngle;
		if (isCompBot) {
			coefficients = controllerCoefficients.compBotTurn;
		} else {
			coefficients = controllerCoefficients.protoBotTurn;
		}
	}

	@Override
	public void startAction() {
		pid = new RobustPID(coefficients, targetAngle, 3, 0.04, Math.toRadians(1));

		timer.reset();
	}

	@Override
	public void runAction() {
		double output = pid.calculateLinearAngle(robot.odometry.subsystemState().getAngleRadians());
		robot.driveTrain.robotRelative(0, output);
		Dashboard.packet.put("power",output);
		Dashboard.packet.put("error",pid.getError());
		isComplete = ((pid.isComplete()) && pid.isStable()) || pid.isBasicallyStopped();
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
