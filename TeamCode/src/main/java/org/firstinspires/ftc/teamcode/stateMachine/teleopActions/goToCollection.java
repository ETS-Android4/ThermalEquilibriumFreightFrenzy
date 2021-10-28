package org.firstinspires.ftc.teamcode.stateMachine.teleopActions;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.stateMachine.teleopAction;
import org.firstinspires.ftc.teamcode.subsystems.robot;
import org.firstinspires.ftc.teamcode.subsystems.scoringMechanisms.deposit;

public class goToCollection implements teleopAction {

	protected robot robot;
	protected Gamepad gamepad1;
	protected Gamepad gamepad2;

	public goToCollection(robot robot, Gamepad gamepad1, Gamepad gamepad2) {
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
			robot.Deposit.setState(deposit.depositStates.COLLECTION);
		}
	}

	@Override
	public boolean isComplete() {
		return false;
	}

	@Override
	public boolean shouldRun() {
		return robot.Deposit.getState().equals(deposit.depositStates.IN);
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
