package org.firstinspires.ftc.teamcode.commandBase.autoActions.DrivetrainControl.LastResort;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Geometry.Vector3D;
import org.firstinspires.ftc.teamcode.commandBase.action;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

public class TimeBasedMove implements action {

	Robot robot;
	Vector3D robotPowers;
	ElapsedTime timer = new ElapsedTime();
	double cutoffTime = 0;

	public TimeBasedMove(Robot robot, Vector3D robotPowers, double cutoffTime) {
		this.robot = robot;
		this.robotPowers = robotPowers;
		this.cutoffTime = cutoffTime;
	}

	@Override
	public void startAction() {
		timer.reset();
	}

	@Override
	public void runAction() {

		if (timer.seconds() > cutoffTime) {
			robot.driveTrain.STOP();
		} else {
			robot.driveTrain.robotRelative(robotPowers);
		}

	}

	@Override
	public void stopAction() {
		robot.driveTrain.STOP();
	}

	@Override
	public boolean isActionComplete() {
		return false;
	}

	@Override
	public boolean isActionPersistent() {
		return true;
	}

	@Override
	public boolean isAMultipleAction() {
		return false;
	}
}
