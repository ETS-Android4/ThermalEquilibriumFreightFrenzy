package org.firstinspires.ftc.teamcode.stateMachine.teleopActions;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.stateMachine.teleopAction;
import org.firstinspires.ftc.teamcode.subsystems.robot;
import org.firstinspires.ftc.teamcode.subsystems.scoringMechanisms.deposit;

import static org.firstinspires.ftc.teamcode.subsystems.scoringMechanisms.deposit.depositStates.IN;

public class commandDeposit implements teleopAction {

	protected final double DEPOSIT_DURATION = 300;
	protected org.firstinspires.ftc.teamcode.subsystems.robot robot;
	protected Gamepad gamepad1;
	protected Gamepad gamepad2;
	protected deposit.depositStates state = IN;
	protected deposit.depositStates stateOfTarget = IN;
	protected boolean initialRunHasOccurred = false;
	protected boolean isRunning = false;
	protected boolean actionIsComplete = false;
	protected ElapsedTime timer = new ElapsedTime();

	public commandDeposit(robot robot, Gamepad gamepad1, Gamepad gamepad2) {
		this.robot = robot;
		this.gamepad1 = gamepad1;
		this.gamepad2 = gamepad2;
	}

	@Override
	public void initialRun() {
		timer.reset();
		initialRunHasOccurred = true;
	}

	@Override
	public void periodic() {

		switch (state) {
			case DISARMED:
				break;

			case IN:
				// cringe imo
				if (timer.milliseconds() > DEPOSIT_DURATION) {
					actionIsComplete = true;
					reset();
				}

				break;
			case AT_HIGH:
			case AT_MID:
			case AT_LOW:
				transitionToDeposit();
				break;
			case DEPOSITING:
				if (timer.milliseconds() > DEPOSIT_DURATION) {
					state = IN;
					timer.reset();
				}
				break;
		}

		System.out.println("current state is " + state);

		robot.Deposit.setState(state);
	}

	protected void transitionToDeposit() {
		if (robot.Deposit.isSlideWithinTolerance()) {
			state = deposit.depositStates.DEPOSITING;
			timer.reset();
		}
	}

	@Override
	public boolean isComplete() {
		return false;
	}

	@Override
	public boolean shouldRun() {


		if (isRunning) {
			System.out.println("is currently running!!");
			return true;
		}

		boolean high = highButton();
		boolean mid = midButton();
		boolean low = lowButton();

		if (high || mid || low) {
			isRunning = true;

			if (high) {
				state = deposit.depositStates.AT_HIGH;
				System.out.println("high!!");

			} else if (mid) {
				state = deposit.depositStates.AT_MID;
				System.out.println("mid!!");

			} else {
				state = deposit.depositStates.AT_LOW;
				System.out.println("low!!");

			}
		}

		robot.Deposit.setState(state);

		return isRunning;
	}

	@Override
	public void reset() {

		isRunning = false;
		initialRunHasOccurred = false;
		actionIsComplete = false;
		stateOfTarget = IN;
		timer.reset();
	}

	@Override
	public boolean hasPerformedInitialRun() {
		return initialRunHasOccurred;
	}


	public boolean highButton() {
		return gamepad1.triangle;
	}

	public boolean midButton() {
		return gamepad1.square;
	}

	public boolean lowButton() {
		return gamepad1.cross;
	}
}
