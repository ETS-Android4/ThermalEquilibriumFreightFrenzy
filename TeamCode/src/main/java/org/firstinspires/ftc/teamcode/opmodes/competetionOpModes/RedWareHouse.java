package org.firstinspires.ftc.teamcode.opmodes.competetionOpModes;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commandBase.autoActions.DrivetrainControl.AimAtPoint;
import org.firstinspires.ftc.teamcode.commandBase.autoActions.DrivetrainControl.Drive;
import org.firstinspires.ftc.teamcode.commandBase.autoActions.DrivetrainControl.Turn;
import org.firstinspires.ftc.teamcode.commandBase.autoActions.SlideControl.DepositFreight;
import org.firstinspires.ftc.teamcode.commandBase.autoActions.SlideControl.GoToBottomDeposit;
import org.firstinspires.ftc.teamcode.commandBase.autoActions.SlideControl.GoToHighDeposit;
import org.firstinspires.ftc.teamcode.commandBase.autoActions.SlideControl.GoToInState;
import org.firstinspires.ftc.teamcode.commandBase.autoActions.SlideControl.GoToMidDeposit;
import org.firstinspires.ftc.teamcode.geometry.Vector3D;
import org.firstinspires.ftc.teamcode.templateOpModes.BaseAuto;

@Autonomous
public class RedWareHouse extends BaseAuto {
	public Vector3D startPosition;
	public Vector3D goal;
	public double DISTANCE_BACK_FROM_GOAL = - 13;
	@Override
	public void setStartingPosition() {
		startPosition = new Vector3D(9, -56, Math.toRadians(-90));
		goal = new Vector3D(-12, -20, 0);
		robot.setRobotPose(startPosition);
	}

	@Override
	public void setVisionSettings() {
		setVisionForLeftVisible();
	}

	@Override
	public void addActions() {
		setSlidePosition();
		goToGoal();
		place();
		retreat();
		park();
	}

	public void setSlidePosition() {
		switch (TSEPosition) {
			case LEFT:
				// scoring level 1
				actions.add(new GoToBottomDeposit(robot));
				break;
			case MIDDLE:
				// scoring level 2
				actions.add(new GoToMidDeposit(robot));
				break;
			case RIGHT:
				// scoring level 3
				actions.add(new GoToHighDeposit(robot));
				break;
		}
	}
	public void goToGoal() {
		actions.add(new Drive(robot, -15));
		actions.add(new AimAtPoint(robot, goal, false, true));
		actions.add(new Drive(robot, goal, -1, DISTANCE_BACK_FROM_GOAL - 1));
	}

	public void place() {
		actions.add(new DepositFreight(robot));
	}

	public void retreat() {
		actions.add(new Drive(robot, 20));
		actions.add(new GoToInState(robot));
	}

	public void park() {
		actions.add(new Turn(robot, Math.toRadians(0)));
		actions.add(new Drive(robot, 40));
	}
}
