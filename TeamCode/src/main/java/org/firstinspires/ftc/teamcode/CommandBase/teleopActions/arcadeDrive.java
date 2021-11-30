package org.firstinspires.ftc.teamcode.CommandBase.teleopActions;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.CommandBase.teleopAction;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

public class arcadeDrive implements teleopAction {

	protected Robot robot;
	protected Gamepad gamepad1;
	protected Gamepad gamepad2;

	protected double scalar = 1;
	protected final double slow_scalar = 0.5;

	public arcadeDrive(Robot robot, Gamepad gamepad1, Gamepad gamepad2) {
		this.robot = robot;
		this.gamepad1 = gamepad1;
		this.gamepad2 = gamepad2;
	}

	@Override
	public void initialRun() {

	}

	@Override
	public void periodic() {
		if (gamepad1.right_bumper) {
			scalar = 0.5;
		} else {
			scalar = 1;
		}
		robot.driveTrain.robotRelativeRaw(-gamepad1.right_stick_y * scalar,
				 					     gamepad1.left_stick_x * scalar);
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
