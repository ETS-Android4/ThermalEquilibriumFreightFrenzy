package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.baseopmode.baseAuto;
import org.firstinspires.ftc.teamcode.geometry.Vector3D;
import org.firstinspires.ftc.teamcode.stateMachine.actions.aimAtPoint;
import org.firstinspires.ftc.teamcode.stateMachine.actions.basedDrive;
import org.firstinspires.ftc.teamcode.stateMachine.actions.delay;
import org.firstinspires.ftc.teamcode.stateMachine.actions.depositFreight;
import org.firstinspires.ftc.teamcode.stateMachine.actions.goToHighDeposit;
import org.firstinspires.ftc.teamcode.stateMachine.actions.goToInState;
import org.firstinspires.ftc.teamcode.stateMachine.actions.turnOffIntake;
import org.firstinspires.ftc.teamcode.stateMachine.actions.turnOnIntake;

@Autonomous
public class redDrippedAfAuto extends baseAuto {
	public Vector3D startPosition = new Vector3D(9, -56, Math.toRadians(-90));
	public Vector3D goalPosition = new Vector3D(-12, -24, 0);
	public Vector3D pickupPosition = new Vector3D(60, -50, 0);

	public void setStartingPosition() {
		robot.setRobotPose(startPosition);
	}

	public void addActions() {
//		actions.add(new goToHighDeposit(robot));
//		// go to deposit position
//		actions.add(new basedDrive(robot,-startPosition.distanceToPose(depositPosition)));
//		// aim towards goal
//		actions.add(new aimAtPoint(robot,goalPosition,false,true));
//		// drive to goal
//		actions.add(new basedDrive(robot,-depositPosition.distanceToPose(goalPosition) / 4));
//		// place cube
//		actions.add(new depositFreight(robot));
//
//		actions.add(new basedDrive(robot,depositPosition.distanceToPose(goalPosition) / 3));
//		actions.add(new goToInState(robot));
//		actions.add(new basedTurn(robot,Math.toRadians(0)));
//		actions.add(new basedDrive(robot, 50));
//
//		actions.add(new aimAtPoint(robot,pickupPosition));
//		actions.add(new turnOnIntake(robot));
//		actions.add(new basedDrive(robot, pickupPosition, 0.8));
//		actions.add(new turnOffIntake(robot));
		actions.add(new goToHighDeposit(robot));
		actions.add(new basedDrive(robot, -5));
		actions.add(new aimAtPoint(robot, goalPosition, false, true));
		actions.add(new basedDrive(robot, goalPosition, -0.8));
		actions.add(new depositFreight(robot));
		actions.add(new basedDrive(robot, startPosition, 0.2));

		actions.add(new aimAtPoint(robot, pickupPosition));
		actions.add(new goToInState(robot));
		actions.add(new basedDrive(robot, pickupPosition, 0.8));
		actions.add(new turnOnIntake(robot));
		actions.add(new basedDrive(robot, pickupPosition, 0.6));
		actions.add(new delay(300));
		actions.add(new turnOffIntake(robot));
		actions.add(new aimAtPoint(robot, goalPosition, false, true));
		actions.add(new basedDrive(robot, goalPosition, -0.8));
		actions.add(new goToHighDeposit(robot));
		actions.add(new depositFreight(robot));
		actions.add(new aimAtPoint(robot, pickupPosition));
		actions.add(new goToInState(robot));
		actions.add(new basedDrive(robot, pickupPosition, 0.8));


	}
}
