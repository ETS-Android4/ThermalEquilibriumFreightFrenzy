package org.firstinspires.ftc.teamcode.stateMachine.actions;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.classicalControl.DifferentialDriveController;
import org.firstinspires.ftc.teamcode.geometry.Vector3D;
import org.firstinspires.ftc.teamcode.stateMachine.action;
import org.firstinspires.ftc.teamcode.subsystems.robot;

public class driveToPositionDifferential implements action {

	protected robot robot;
	protected Vector3D targetPose;
	protected boolean isComplete = false;
	DifferentialDriveController controller;
	ElapsedTime timer = new ElapsedTime();
	double timeOutTime;

	public driveToPositionDifferential(robot robot, Vector3D targetPose) {
		this.robot = robot;
		this.targetPose = targetPose;
		controller = new DifferentialDriveController(robot);

	}

	@Override
	public void startAction() {

		timeOutTime = (targetPose.distanceToPose(robot.getRobotPose()) / DifferentialDriveController.MAX_VELO) * 5;

		timer.reset();

	}

	@Override
	public void runAction() {
		isComplete = controller.driveToPosition(targetPose) || timer.seconds() > timeOutTime;
	}

	@Override
	public void stopAction() {
		robot.driveTrain.STOP();
	}

	@Override
	public boolean isActionComplete() {
		return isComplete;
	}
}
