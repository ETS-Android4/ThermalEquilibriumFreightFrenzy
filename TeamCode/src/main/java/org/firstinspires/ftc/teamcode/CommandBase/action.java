package org.firstinspires.ftc.teamcode.CommandBase;

public interface action {

	void startAction();

	void runAction();

	void stopAction();

	boolean isActionComplete();

	boolean isActionPersistent();

}
