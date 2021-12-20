package org.firstinspires.ftc.teamcode.commandBase.teleopActions;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.commandBase.teleopAction;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

public class ArcadeDrive implements teleopAction {

	protected Robot robot;
	protected Gamepad gamepad1;
	protected Gamepad gamepad2;

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

		robot.driveTrain.robotRelativeRaw(-gamepad1.right_stick_y,
				 					       gamepad1.left_stick_x);
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
