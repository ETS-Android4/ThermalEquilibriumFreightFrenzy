package org.firstinspires.ftc.teamcode.CommandBase.actions;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CommandBase.action;


public class delay implements action {

	protected boolean isComplete = false;
	double durationMs;
	ElapsedTime timer = new ElapsedTime();

	public delay(double durationMs) {
		this.durationMs = durationMs;
	}

	@Override
	public void startAction() {
		timer.reset();
	}

	@Override
	public void runAction() {

		if (timer.milliseconds() > durationMs) {
			isComplete = true;
		}

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
