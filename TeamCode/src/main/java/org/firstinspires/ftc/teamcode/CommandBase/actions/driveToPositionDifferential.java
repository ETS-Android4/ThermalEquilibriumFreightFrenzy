package org.firstinspires.ftc.teamcode.CommandBase.actions;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.classicalControl.DifferentialDriveController;
import org.firstinspires.ftc.teamcode.Geometry.Vector3D;
import org.firstinspires.ftc.teamcode.CommandBase.action;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

public class driveToPositionDifferential implements action {

	protected Robot robot;
	protected Vector3D targetPose;
	protected boolean isComplete = false;
	protected boolean withinPoseDistance = false;
	protected boolean timeOutExceeded = false;
	DifferentialDriveController controller;
	ElapsedTime timer = new ElapsedTime();
	double timeOutTime;

	public driveToPositionDifferential(Robot robot, Vector3D targetPose) {
		this.robot = robot;
		this.targetPose = targetPose;
		controller = new DifferentialDriveController(robot);

	}

	@Override
	public void startAction() {

		timeOutTime = (targetPose.distanceToPose(robot.getRobotPose()) / DifferentialDriveController.MAX_VELO) * 7;

		timer.reset();

	}

	@Override
	public void runAction() {
		withinPoseDistance = controller.driveToPosition(targetPose);
		timeOutExceeded = timer.seconds() > timeOutTime;
		isComplete = withinPoseDistance || timeOutExceeded;
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
