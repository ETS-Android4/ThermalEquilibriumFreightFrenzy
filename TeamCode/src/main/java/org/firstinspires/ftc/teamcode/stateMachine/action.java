package org.firstinspires.ftc.teamcode.stateMachine;

public interface action {

	void startAction();

	void runAction();

	void stopAction();

	boolean isActionComplete();

	boolean isActionPersistent();

}
