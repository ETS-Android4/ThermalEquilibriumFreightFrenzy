package org.firstinspires.ftc.teamcode.stateMachine.actions;

import org.firstinspires.ftc.teamcode.stateMachine.action;
import org.firstinspires.ftc.teamcode.subsystems.robot;
import org.firstinspires.ftc.teamcode.subsystems.scoringMechanisms.deposit;
import org.firstinspires.ftc.teamcode.subsystems.scoringMechanisms.intake;

public class turnOnIntake implements action {
	protected robot robot;
	protected boolean isComplete = false;

	public turnOnIntake(robot robot) {
		this.robot = robot;
	}

	@Override
	public void startAction() {
		robot.bucketSys.setState(deposit.depositStates.COLLECTION);
		robot.Deposit.setState(deposit.depositStates.COLLECTION);
		robot.Intake.setState(intake.intakeStates.ON);
		isComplete = true;
	}

	@Override
	public void runAction() {

	}

	@Override
	public void stopAction() {
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
