package org.firstinspires.ftc.teamcode.CommandBase.teleopActions;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.CommandBase.teleopAction;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.scoringMechanisms.Deposit;

public class goToCollection implements teleopAction {

	protected Robot robot;
	protected Gamepad gamepad1;
	protected Gamepad gamepad2;

	public goToCollection(Robot robot, Gamepad gamepad1, Gamepad gamepad2) {
		this.robot = robot;
		this.gamepad1 = gamepad1;
		this.gamepad2 = gamepad2;
	}

	@Override
	public void initialRun() {

	}

	@Override
	public void periodic() {
		if (collectionButton()) {
			robot.Deposit.setState(Deposit.depositStates.COLLECTION);
		}
	}

	@Override
	public boolean isComplete() {
		return false;
	}

	@Override
	public boolean shouldRun() {
		return robot.Deposit.getState().equals(Deposit.depositStates.IN);
	}

	@Override
	public void reset() {

	}

	@Override
	public boolean hasPerformedInitialRun() {
		return true;
	}

	public boolean collectionButton() {
		return gamepad1.dpad_up;
	}
}
