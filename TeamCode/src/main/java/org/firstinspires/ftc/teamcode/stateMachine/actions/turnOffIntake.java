package org.firstinspires.ftc.teamcode.stateMachine.actions;

import org.firstinspires.ftc.teamcode.stateMachine.action;
import org.firstinspires.ftc.teamcode.subsystems.robot;
import org.firstinspires.ftc.teamcode.subsystems.scoringMechanisms.deposit;
import org.firstinspires.ftc.teamcode.subsystems.scoringMechanisms.intake;

public class turnOffIntake implements action {

	protected boolean isComplete = false;
	protected robot robot;

	public turnOffIntake(robot robot) {
		this.robot = robot;
	}

	@Override
	public void startAction() {

		robot.Intake.setState(intake.intakeStates.OFF);
		robot.Deposit.setState(deposit.depositStates.IN);
		robot.bucketSys.setState(deposit.depositStates.IN);
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
