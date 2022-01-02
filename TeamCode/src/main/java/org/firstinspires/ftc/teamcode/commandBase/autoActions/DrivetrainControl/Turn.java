package org.firstinspires.ftc.teamcode.commandBase.autoActions.DrivetrainControl;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.controls.RobustPID;
import org.firstinspires.ftc.teamcode.controls.controllerCoefficients;
import org.firstinspires.ftc.teamcode.classicalControl.PIDFCoefficients;
import org.firstinspires.ftc.teamcode.commandBase.action;
import org.firstinspires.ftc.teamcode.subsystems.Dashboard;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

import static org.firstinspires.ftc.teamcode.subsystems.Robot.isCompBot;

public class Turn implements action {

	Robot robot;
	RobustPID pid;

	PIDFCoefficients coefficients;
	double targetAngle;
	boolean isComplete = false;
	ElapsedTime timer = new ElapsedTime();

	public Turn(Robot robot, double targetAngle) {
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
		pid = new RobustPID(coefficients, targetAngle, 2, 0.02, Math.toRadians(1));

		timer.reset();
	}

	@RequiresApi(api = Build.VERSION_CODES.N)
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
