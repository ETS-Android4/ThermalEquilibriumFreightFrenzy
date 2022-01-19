package org.firstinspires.ftc.teamcode.commandBase.autoActions.DrivetrainControl;

import org.firstinspires.ftc.teamcode.Controls.MIMOControls.MecanumDriveController;
import org.firstinspires.ftc.teamcode.commandBase.action;
import org.firstinspires.ftc.teamcode.Geometry.Vector3D;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

public class DriveToPosition implements action {

	Robot robot;
	Vector3D referencePose;
	MecanumDriveController controller;

	public DriveToPosition(Robot robot, Vector3D referencePose) {
		this.robot = robot;
		this.referencePose = referencePose;
		controller = new MecanumDriveController();
	}

	@Override
	public void startAction() {
		controller.resetTimer();
	}

	@Override
	public void runAction() {
		Vector3D fieldRelative = controller.calculateSpeedRamped(referencePose, robot.getRobotPose(), robot.getVelocity());
		robot.driveTrain.robotRelative(fieldRelative);
	}

	@Override
	public void stopAction() {
		robot.driveTrain.STOP();
	}

	@Override
	public boolean isActionComplete() {
		return controller.followingIsComplete();
	}

	@Override
	public boolean isActionPersistent() {
		return true;
	}
}
