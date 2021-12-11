package org.firstinspires.ftc.teamcode.opmodes.competetionOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.templateOpModes.BaseAuto;
import org.firstinspires.ftc.teamcode.geometry.Vector3D;
import org.firstinspires.ftc.teamcode.commandBase.autoActions.DrivetrainControl.AimAtPoint;
import org.firstinspires.ftc.teamcode.commandBase.autoActions.Drive;
import org.firstinspires.ftc.teamcode.commandBase.autoActions.Delay;
import org.firstinspires.ftc.teamcode.commandBase.autoActions.SlideControl.GoToInState;
import org.firstinspires.ftc.teamcode.commandBase.autoActions.TurnOffIntake;
import org.firstinspires.ftc.teamcode.commandBase.autoActions.TurnOnIntake;

@Autonomous
public class RedWarehouseAuto extends BaseAuto {
	public Vector3D startPosition = new Vector3D(9, -56, Math.toRadians(-90));
	public Vector3D goalPosition = new Vector3D(-12, -24, 0);
	public Vector3D pickupPosition = new Vector3D(60, -50, 0);

	public void setStartingPosition() {
		robot.setRobotPose(startPosition);
	}

	@Override
	public void setVisionSettings() {

	}

	public void addActions() {

		actions.add(new Drive(robot, -5));
		actions.add(new AimAtPoint(robot, goalPosition, false, true));
		actions.add(new Drive(robot, goalPosition, -0.8));
		//actions.add(new depositFreight(robot));
		actions.add(new Drive(robot, startPosition, 0.2));

		actions.add(new AimAtPoint(robot, pickupPosition));
		actions.add(new GoToInState(robot));
		actions.add(new Drive(robot, pickupPosition, 0.8));
		actions.add(new TurnOnIntake(robot));
		actions.add(new Drive(robot, pickupPosition, 0.6));
		actions.add(new Delay(300));
		actions.add(new TurnOffIntake(robot));
		actions.add(new AimAtPoint(robot, goalPosition, false, true));
		actions.add(new Drive(robot, goalPosition, -0.8));
		//actions.add(new goToHighDeposit(robot));
		//actions.add(new depositFreight(robot));
		actions.add(new AimAtPoint(robot, pickupPosition));
		//actions.add(new goToInState(robot));
		actions.add(new Drive(robot, pickupPosition, 0.8));


	}
}
