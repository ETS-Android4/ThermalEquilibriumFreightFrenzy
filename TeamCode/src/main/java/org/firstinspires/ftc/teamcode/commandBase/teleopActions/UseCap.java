package org.firstinspires.ftc.teamcode.commandBase.teleopActions;

import android.widget.Button;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.commandBase.teleopAction;
import org.firstinspires.ftc.teamcode.gamepadEnhancements.ButtonPress;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

public class UseCap implements teleopAction {

	protected Robot robot;
	protected Gamepad gamepad1;
	protected Gamepad gamepad2;
	ButtonPress nextStatePress = new ButtonPress();
	ButtonPress previousStatePress = new ButtonPress();


	public UseCap(Robot robot, Gamepad gamepad1, Gamepad gamepad2) {
		this.robot = robot;
		this.gamepad1 = gamepad1;
		this.gamepad2 = gamepad2;
	}

	@Override
	public void initialRun() {

	}

	@Override
	public void periodic() {

		nextStatePress.button(gamepad1.dpad_up);
		previousStatePress.button(gamepad1.dpad_right);
		if (nextStatePress.press()) {
			robot.capArm.nextState();
		}
		if (previousStatePress.press()) {
			robot.capArm.previousState();
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
