package org.firstinspires.ftc.teamcode.commandBase.autoActions.SlideControl;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commandBase.action;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.scoringMechanisms.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.scoringMechanisms.Intake;

import static org.firstinspires.ftc.teamcode.subsystems.scoringMechanisms.Deposit.depositStates.AT_HIGH;

/**
 * Move the scoring mechanism to deposit in the high goal.
 *
 * DOES NOT MOVE THE ROBOT TOWARDS THE DEPOSIT LOCATION
 */
public class GoToHighDeposit implements action {

	protected boolean isComplete = false;
	protected Deposit.depositStates state = Deposit.depositStates.GOING_TO_HIGH;
	protected double TIME_FOR_COMPLETION = 300;
	Robot robot;
	ElapsedTime timer = new ElapsedTime();

	public GoToHighDeposit(Robot robot) {
		this.robot = robot;
	}

	@Override
	public void startAction() {

		robot.bucketSys.setState(state);
		robot.Deposit.setState(state);
		robot.Intake.setState(Intake.intakeStates.OFF);

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
