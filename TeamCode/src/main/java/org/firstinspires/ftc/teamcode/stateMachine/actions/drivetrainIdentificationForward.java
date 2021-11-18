package org.firstinspires.ftc.teamcode.stateMachine.actions;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.stateMachine.action;
import org.firstinspires.ftc.teamcode.subsystems.robot;

public class drivetrainIdentificationForward implements action {

	protected robot robot;
	protected double power = 0;
	protected double max_power = 0.75;
	protected double step_time_seconds = 1;
	protected double total_step_time = 2;
	protected boolean isComplete = false;
	ElapsedTime timer = new ElapsedTime();

	public drivetrainIdentificationForward(robot robot) {
		this.robot = robot;
	}

	@Override
	public void startAction() {
		System.out.println("drivetrain forward: power, position, time");
		timer.reset();
	}

	@Override
	public void runAction() {
		double time = timer.seconds();
		if (time > step_time_seconds) {
			power = max_power;
		}
		robot.driveTrain.robotRelative(power, 0);
		System.out.println("drivetrain forward: " + power + ", " + robot.getRobotPose().getX() + ", " + time);
		if (time > total_step_time) {
			isComplete = true;
		}
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
		return false;
	}
}
