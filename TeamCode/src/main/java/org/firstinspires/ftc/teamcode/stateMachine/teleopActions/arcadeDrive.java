package org.firstinspires.ftc.teamcode.stateMachine.teleopActions;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.stateMachine.teleopAction;
import org.firstinspires.ftc.teamcode.subsystems.robot;

public class arcadeDrive implements teleopAction {

	protected robot robot;
	protected Gamepad gamepad1;
	protected Gamepad gamepad2;

	public arcadeDrive(robot robot, Gamepad gamepad1, Gamepad gamepad2) {
		this.robot = robot;
		this.gamepad1 = gamepad1;
		this.gamepad2 = gamepad2;
	}

	@Override
	public void initialRun() {

	}

	@Override
	public void periodic() {
		robot.driveTrain.robotRelative(-gamepad1.left_stick_y, gamepad1.left_stick_x);
	}

	@Override
	public boolean isComplete() {
		return false;
	}

	@Override
	public boolean shouldRun() {
		return true;
	}

	@Override
	public void reset() {

	}

	/**
	 * in this particular case we can simply skip the initial run
	 *
	 * @return true always
	 */
	@Override
	public boolean hasPerformedInitialRun() {
		return true;
	}

	public boolean areJoysticksActive() {
		double joystickTotal = Math.abs(gamepad1.left_stick_x)
				+ Math.abs(gamepad1.right_stick_y)
				+ Math.abs(gamepad1.left_stick_y)
				+ Math.abs(gamepad1.right_stick_x);

		return joystickTotal > 0.05;
	}
}
