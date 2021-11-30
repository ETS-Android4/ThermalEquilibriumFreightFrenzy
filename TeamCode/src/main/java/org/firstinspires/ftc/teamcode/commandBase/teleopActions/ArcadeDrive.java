package org.firstinspires.ftc.teamcode.commandBase.teleopActions;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.commandBase.teleopAction;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

public class ArcadeDrive implements teleopAction {

	protected Robot robot;
	protected Gamepad gamepad1;
	protected Gamepad gamepad2;

	protected double scalar = 1;

	public ArcadeDrive(Robot robot, Gamepad gamepad1, Gamepad gamepad2) {
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

}
