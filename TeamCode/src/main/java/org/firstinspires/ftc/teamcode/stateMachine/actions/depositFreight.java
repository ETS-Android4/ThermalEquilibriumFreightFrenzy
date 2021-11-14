package org.firstinspires.ftc.teamcode.stateMachine.actions;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.stateMachine.action;
import org.firstinspires.ftc.teamcode.subsystems.robot;
import org.firstinspires.ftc.teamcode.subsystems.scoringMechanisms.deposit;

public class depositFreight implements action {

	protected robot robot;
	protected boolean isComplete = false;
	protected ElapsedTime timer = new ElapsedTime();
	protected double depositTime = 300;

	public depositFreight(robot robot) {
		this.robot = robot;
	}

	@Override
	public void startAction() {
		robot.bucketSys.setState(deposit.depositStates.DEPOSITING);
		robot.Deposit.setState(deposit.depositStates.DEPOSITING);
		timer.reset();
	}

	@Override
	public void runAction() {
		if (timer.milliseconds() > depositTime) {
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
