package org.firstinspires.ftc.teamcode.CommandBase.actions;

import org.firstinspires.ftc.teamcode.CommandBase.action;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.scoringMechanisms.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.scoringMechanisms.Intake;

public class turnOnIntake implements action {
	protected Robot robot;
	protected boolean isComplete = false;

	public turnOnIntake(Robot robot) {
		this.robot = robot;
	}

	@Override
	public void startAction() {
		robot.bucketSys.setState(Deposit.depositStates.COLLECTION);
		robot.Deposit.setState(Deposit.depositStates.COLLECTION);
		robot.Intake.setState(Intake.intakeStates.ON);
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
