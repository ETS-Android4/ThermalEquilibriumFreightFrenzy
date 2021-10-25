package org.firstinspires.ftc.teamcode.stateMachine.teleopActions;

import org.firstinspires.ftc.teamcode.stateMachine.teleopAction;
import org.firstinspires.ftc.teamcode.subsystems.robot;

import static org.firstinspires.ftc.teamcode.subsystems.scoringMechanisms.deposit.depositStates.IN;
import static org.firstinspires.ftc.teamcode.subsystems.scoringMechanisms.intake.intakeStates.OFF;
import static org.firstinspires.ftc.teamcode.subsystems.scoringMechanisms.intake.intakeStates.ON;

public class toggleIntake implements teleopAction {


	protected robot robot;


	public toggleIntake(robot robot) {
		this.robot = robot;

	}

	@Override
	public void initialRun() {

	}

	@Override
	public void periodic() {
		if (robot.Deposit.getState().equals(IN)) {
			robot.Intake.setState(ON);
		} else robot.Intake.setState(OFF);
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
