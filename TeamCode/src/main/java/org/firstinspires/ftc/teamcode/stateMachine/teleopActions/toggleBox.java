package org.firstinspires.ftc.teamcode.stateMachine.teleopActions;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.stateMachine.teleopAction;
import org.firstinspires.ftc.teamcode.subsystems.robot;
import org.firstinspires.ftc.teamcode.subsystems.scoringMechanisms.deposit;

public class toggleBox implements teleopAction {

	protected robot robot;
	protected Gamepad gamepad1;
	protected Gamepad gamepad2;


	public toggleBox(robot robot, Gamepad gamepad1, Gamepad gamepad2) {
		this.robot = robot;
		this.gamepad1 = gamepad1;
		this.gamepad2 = gamepad2;
	}


	@Override
	public void initialRun() {

	}

	@Override
	public void periodic() {
		if (gamepad1.left_bumper) {
			robot.bucketSys.setState(deposit.depositStates.DEPOSITING);
		} else {
			robot.bucketSys.setState(deposit.depositStates.IN);
		}
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

	@Override
	public boolean hasPerformedInitialRun() {
		return true;
	}
}
