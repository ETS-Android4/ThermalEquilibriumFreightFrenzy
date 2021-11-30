package org.firstinspires.ftc.teamcode.CommandBase.actions;

import org.firstinspires.ftc.teamcode.CommandBase.action;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.scoringMechanisms.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.scoringMechanisms.Intake;

public class turnOffIntake implements action {

	protected boolean isComplete = false;
	protected Robot robot;

	public turnOffIntake(Robot robot) {
		this.robot = robot;
	}

	@Override
	public void startAction() {

		robot.Intake.setState(Intake.intakeStates.OFF);
		robot.Deposit.setState(Deposit.depositStates.IN);
		robot.bucketSys.setState(Deposit.depositStates.IN);
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
