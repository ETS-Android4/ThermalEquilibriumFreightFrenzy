package org.firstinspires.ftc.teamcode.opmodes.competetionOpModes;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commandBase.autoActions.DrivetrainControl.AimAtPoint;
import org.firstinspires.ftc.teamcode.commandBase.autoActions.DrivetrainControl.Drive;
import org.firstinspires.ftc.teamcode.commandBase.autoActions.DrivetrainControl.Turn;
import org.firstinspires.ftc.teamcode.commandBase.autoActions.Ducks.SetDuckWheel;
import org.firstinspires.ftc.teamcode.commandBase.autoActions.Intake.TurnOffIntake;
import org.firstinspires.ftc.teamcode.commandBase.autoActions.Intake.TurnOnIntake;
import org.firstinspires.ftc.teamcode.commandBase.autoActions.Misc.Delay;
import org.firstinspires.ftc.teamcode.commandBase.autoActions.Misc.MoveCapArm;
import org.firstinspires.ftc.teamcode.commandBase.autoActions.SlideControl.DepositFreight;
import org.firstinspires.ftc.teamcode.commandBase.autoActions.SlideControl.GoToBottomDeposit;
import org.firstinspires.ftc.teamcode.commandBase.autoActions.SlideControl.GoToHighDeposit;
import org.firstinspires.ftc.teamcode.commandBase.autoActions.SlideControl.GoToInState;
import org.firstinspires.ftc.teamcode.commandBase.autoActions.SlideControl.GoToMidDeposit;
import org.firstinspires.ftc.teamcode.geometry.Vector3D;
import org.firstinspires.ftc.teamcode.subsystems.DuckWheel;
import org.firstinspires.ftc.teamcode.subsystems.scoringMechanisms.CapArm;
import org.firstinspires.ftc.teamcode.templateOpModes.BaseAuto;


@Autonomous
public class BlueDuckAuto extends BaseAuto {

	public Vector3D startPosition;
	public Vector3D goalPosition;
	public Vector3D carousel;
	public Vector3D park;
	public Vector3D leftCapStone;
	public Vector3D middleCapstone;
	public Vector3D rightCapstone;
	public double exitCarouselAngle;
	public double DISTANCE_BACK_FROM_GOAL = -13;
	public double PUSH_WHEEL_AGAINST_CAROUSEL = 4;
	public double DISTANCE_TO_COLLECT_DUCK = 3;
	public double DUCK_DURATION = 2500;
	public double GO_TO_PARK_DISTANCE_FIRST_MOVE = 14;

	@Override
	public void setStartingPosition() {
		startPosition = new Vector3D(-39, 56, Math.toRadians(90));
		goalPosition = new Vector3D(-12, 20, 0);
		carousel = new Vector3D(-60, 55, 0);
		park = new Vector3D(-60, 29, 0);
		leftCapStone = new Vector3D(-48 - 2, 36, 0);
		middleCapstone = new Vector3D(-48 - 12, 36, 0);
		rightCapstone = new Vector3D(-48 - 20, 36, 0);
		exitCarouselAngle = Math.toRadians(105);
		robot.setRobotPose(startPosition);

	}

	@Override
	public void setVisionSettings() {
		setVisionForRightVisible();
	}

	@Override
	public void addActions() {

		actions.add(new Delay(3000));

		actions.add(new MoveCapArm(robot, CapArm.ArmStates.ANTENNA));

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
		// deploy slides

		// drive to goal to deposit

		switch (TSEPosition) {
			case LEFT:
				actions.add(new Drive(robot, -15));
				actions.add(new AimAtPoint(robot, goalPosition, false, true));
				actions.add(new Drive(robot, goalPosition, -1, DISTANCE_BACK_FROM_GOAL + 1));
				actions.add(new DepositFreight(robot));

				// drive to carousel
				actions.add(new Drive(robot, carousel, 1, -3));
				actions.add(new GoToInState(robot));
				actions.add(new TurnOnIntake(robot, false));
				actions.add(new Turn(robot, startPosition.getAngleRadians()));

				// push wheel against carousel
				actions.add(new SetDuckWheel(robot, DuckWheel.DuckWheelState.OTHER_ON));
				actions.add(new Drive(robot, PUSH_WHEEL_AGAINST_CAROUSEL));
				actions.add(new Delay(DUCK_DURATION));
				actions.add(new SetDuckWheel(robot, DuckWheel.DuckWheelState.OFF));

				actions.add(new Drive(robot, park, -1));  // the one here is positive so it drives the intake side forward
				actions.add(new Turn(robot, startPosition.getAngleRadians()));
				break;
			case MIDDLE:
				actions.add(new Drive(robot, -24));
				actions.add(new Drive(robot, goalPosition, -1, DISTANCE_BACK_FROM_GOAL -2));
				actions.add(new DepositFreight(robot));
				actions.add(new Delay(500));

				// drive to carousel
				actions.add(new Drive(robot, carousel, 1, -3));
				actions.add(new GoToInState(robot));
				actions.add(new Turn(robot, startPosition.getAngleRadians()));

				// push wheel against carousel
				actions.add(new SetDuckWheel(robot, DuckWheel.DuckWheelState.OTHER_ON));
				actions.add(new Drive(robot, PUSH_WHEEL_AGAINST_CAROUSEL));
				actions.add(new Delay(DUCK_DURATION));
				actions.add(new SetDuckWheel(robot, DuckWheel.DuckWheelState.OFF));

				// go to park
				actions.add(new Drive(robot, park, -1));  // the one here is positive so it drives the intake side forward
				break;
			case RIGHT:

				actions.add(new Drive(robot, -15));
				actions.add(new AimAtPoint(robot, goalPosition, false, true));
				actions.add(new Drive(robot, goalPosition, -1, DISTANCE_BACK_FROM_GOAL + 1));
				actions.add(new DepositFreight(robot));

				// drive to carousel
				actions.add(new Drive(robot, carousel, 1, -3));
				actions.add(new GoToInState(robot));
				actions.add(new TurnOnIntake(robot, false));
				actions.add(new Turn(robot, startPosition.getAngleRadians()));

				// push wheel against carousel
				actions.add(new SetDuckWheel(robot, DuckWheel.DuckWheelState.OTHER_ON));
				actions.add(new TurnOnIntake(robot, true));
				actions.add(new Drive(robot, PUSH_WHEEL_AGAINST_CAROUSEL));
				actions.add(new Delay(DUCK_DURATION + 1));
				actions.add(new SetDuckWheel(robot, DuckWheel.DuckWheelState.OFF));
				actions.add(new Turn(robot, exitCarouselAngle));
				actions.add(new Drive(robot, DISTANCE_TO_COLLECT_DUCK));
				actions.add(new TurnOffIntake(robot));

//				// deposit the duck
//				actions.add(new GoToHighDeposit(robot));
//				actions.add(new Drive(robot, goalPosition, -1, DISTANCE_BACK_FROM_GOAL -.5));
//
//				// actions.add(new AimAtPoint(robot,goalPosition,false, true));
//				actions.add(new DepositFreight(robot));

				// go to park

				actions.add(new Drive(robot, GO_TO_PARK_DISTANCE_FIRST_MOVE));
				actions.add(new Turn(robot, Math.toRadians(-180)));
				actions.add(new GoToInState(robot));
				actions.add(new Drive(robot, park, 1));  // the one here is positive so it drives the intake side forward

				break;

		}

		actions.add(new MoveCapArm(robot, CapArm.ArmStates.IN));

	}
}