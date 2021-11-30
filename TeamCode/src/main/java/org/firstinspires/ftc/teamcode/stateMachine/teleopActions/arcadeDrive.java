package org.firstinspires.ftc.teamcode.stateMachine.teleopActions;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.stateMachine.teleopAction;
import org.firstinspires.ftc.teamcode.subsystems.robot;

public class arcadeDrive implements teleopAction {

	protected robot robot;
	protected Gamepad gamepad1;
	protected Gamepad gamepad2;

	protected double scalar = 1;
	protected final double slow_scalar = 0.5;

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
		if (slowModeButton()) {
			scalar = slow_scalar;
		} else {
			scalar = 1;
		}
		robot.driveTrain.robotRelativeRawClipped(-gamepad1.right_stick_y,
				 					     gamepad1.left_stick_x,scalar);
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

	public boolean slowModeButton() {
		return gamepad1.right_bumper;
	}
}
