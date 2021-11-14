package org.firstinspires.ftc.teamcode.stateMachine.actions;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.stateMachine.action;
import org.firstinspires.ftc.teamcode.subsystems.robot;
import org.firstinspires.ftc.teamcode.subsystems.scoringMechanisms.deposit;
import org.firstinspires.ftc.teamcode.subsystems.scoringMechanisms.intake;

import static org.firstinspires.ftc.teamcode.subsystems.scoringMechanisms.deposit.depositStates.AT_HIGH;
import static org.firstinspires.ftc.teamcode.subsystems.scoringMechanisms.deposit.depositStates.GOING_IN;
import static org.firstinspires.ftc.teamcode.subsystems.scoringMechanisms.deposit.depositStates.IN;

public class goToInState implements action {

	private static final double DEPOSIT_DURATION = 370;
	protected boolean isComplete = false;
	protected deposit.depositStates state = deposit.depositStates.DEPOSITING;
	protected double TIME_FOR_COMPLETION = 300;
	org.firstinspires.ftc.teamcode.subsystems.robot robot;
	ElapsedTime timer = new ElapsedTime();

	public goToInState(robot robot) {
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
				isComplete = true;
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
				timer.reset();
				state = GOING_IN;
				break;
			case GOING_IN:
				if (timer.milliseconds() > DEPOSIT_DURATION * 2) {
					state = IN;
					timer.reset();
				}
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
