package org.firstinspires.ftc.teamcode.commandBase.autoActions.Misc;

import org.firstinspires.ftc.teamcode.commandBase.action;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.scoringMechanisms.CapArm;

public class MoveCapArm implements action {

	Robot robot;

	boolean isComplete = false;

	CapArm.ArmStates targetState;

	public MoveCapArm(Robot robot, CapArm.ArmStates targetState) {
		this.robot = robot;
		this.targetState = targetState;
	}

	@Override
	public void startAction() {
		this.robot.capArm.setState(targetState);
		this.isComplete = true;
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
