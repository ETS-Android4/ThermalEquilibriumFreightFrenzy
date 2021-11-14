package org.firstinspires.ftc.teamcode.stateMachine.actions;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.stateMachine.action;
import org.firstinspires.ftc.teamcode.subsystems.robot;
import org.firstinspires.ftc.teamcode.subsystems.scoringMechanisms.deposit;
import org.firstinspires.ftc.teamcode.subsystems.scoringMechanisms.intake;

import static org.firstinspires.ftc.teamcode.subsystems.scoringMechanisms.deposit.depositStates.AT_HIGH;

public class goToHighDeposit implements action {

	protected boolean isComplete = false;
	protected deposit.depositStates state = deposit.depositStates.GOING_TO_HIGH;
	protected double TIME_FOR_COMPLETION = 300;
	robot robot;
	ElapsedTime timer = new ElapsedTime();

	public goToHighDeposit(robot robot) {
		this.robot = robot;
	}

	@Override
	public void startAction() {

		robot.bucketSys.setState(state);
		robot.Deposit.setState(state);
		robot.Intake.setState(intake.intakeStates.OFF);

	}

	@Override
	public void runAction() {
		switch (state) {
			case DISARMED:
				break;
			case IN:

				break;
			case COLLECTION:
				break;
			case GOING_TO_HIGH:
				if (robot.Deposit.tolerantEnoughForDeploy()) {
					state = AT_HIGH;
				}
				timer.reset();
				break;
			case GOING_TO_MID:
				break;
			case GOING_TO_LOW:
				break;
			case AT_HIGH:
				if (timer.milliseconds() > TIME_FOR_COMPLETION) {
					isComplete = true;
				}
				break;
			case AT_MID:
				break;
			case AT_LOW:
				break;
			case DEPOSITING:
				break;
			case GOING_IN:
				break;
		}

		robot.bucketSys.setState(state);
		robot.Deposit.setState(state);
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
