package org.firstinspires.ftc.teamcode.CommandBase.teleopActions;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.CommandBase.teleopAction;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

import static org.firstinspires.ftc.teamcode.subsystems.scoringMechanisms.Deposit.depositStates.COLLECTION;
import static org.firstinspires.ftc.teamcode.subsystems.scoringMechanisms.Intake.intakeStates.OFF;
import static org.firstinspires.ftc.teamcode.subsystems.scoringMechanisms.Intake.intakeStates.ON;
import static org.firstinspires.ftc.teamcode.subsystems.scoringMechanisms.Intake.intakeStates.REVERSE;

public class toggleIntake implements teleopAction {


	protected Robot robot;
	protected Gamepad gamepad1;
	protected Gamepad gamepad2;


	public toggleIntake(Robot robot, Gamepad gamepad1, Gamepad gamepad2) {
		this.robot = robot;
		this.gamepad1 = gamepad1;
		this.gamepad2 = gamepad2;
	}

	@Override
	public void initialRun() {

	}

	@Override
	public void periodic() {

		if (robot.Deposit.getState().equals(COLLECTION) || gamepad1.right_trigger > 0.5) {
			robot.Intake.setState(ON);
		} else robot.Intake.setState(OFF);

		if (gamepad1.left_trigger > 0.5) {
			robot.Intake.setState(REVERSE);
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
