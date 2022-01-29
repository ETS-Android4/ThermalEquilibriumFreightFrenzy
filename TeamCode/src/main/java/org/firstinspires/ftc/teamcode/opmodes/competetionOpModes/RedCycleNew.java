package org.firstinspires.ftc.teamcode.opmodes.competetionOpModes;

import android.service.quicksettings.Tile;

import com.acmerobotics.roadrunner.drive.Drive;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Geometry.Vector3D;
import org.firstinspires.ftc.teamcode.commandBase.action;
import org.firstinspires.ftc.teamcode.commandBase.autoActions.DrivetrainControl.DriveToIntake;
import org.firstinspires.ftc.teamcode.commandBase.autoActions.DrivetrainControl.DriveToPosition;
import org.firstinspires.ftc.teamcode.commandBase.autoActions.Intake.DeployIntake;
import org.firstinspires.ftc.teamcode.commandBase.autoActions.Intake.TurnOffIntake;
import org.firstinspires.ftc.teamcode.commandBase.autoActions.Intake.TurnOnIntake;
import org.firstinspires.ftc.teamcode.commandBase.autoActions.Misc.Delay;
import org.firstinspires.ftc.teamcode.commandBase.autoActions.Misc.MutlipleAction;
import org.firstinspires.ftc.teamcode.commandBase.autoActions.Misc.OffsetOdom;
import org.firstinspires.ftc.teamcode.commandBase.autoActions.SlideControl.DepositFreight;
import org.firstinspires.ftc.teamcode.commandBase.autoActions.SlideControl.GoToHighDeposit;
import org.firstinspires.ftc.teamcode.commandBase.autoActions.SlideControl.GoToInState;
import org.firstinspires.ftc.teamcode.opmodes.testOpModes.MultipleActionExample;
import org.firstinspires.ftc.teamcode.templateOpModes.BaseAuto;
import org.opencv.core.Mat;

import java.util.Vector;

@Autonomous
public class RedCycleNew extends BaseAuto {

	Vector3D start = new Vector3D(TILE / 2.0, -TILE * 3 + 8.375, Math.toRadians(-90));

	Vector3D depositPosition1 = new Vector3D(+ 2,-TILE * 2 + 4 ,Math.toRadians(-55));

	Vector3D readyForCollection1 = new Vector3D(TILE - 12, -TILE * 3 + 7,  Math.toRadians(0));

	Vector3D collect1 = new Vector3D(TILE * 2 - 8,-TILE * 3 + 7, Math.toRadians(0));
	Vector3D collect2 = new Vector3D(TILE * 2 - 4,-TILE * 3 + 7, Math.toRadians(0));
	Vector3D collect3 = new Vector3D(TILE * 2 - 1,-TILE * 3 + 7, Math.toRadians(0));

	Vector3D readyForPark = new Vector3D(TILE / 3, -TILE * 3 + 11, Math.toRadians(0));

	Vector3D gapPose = new Vector3D(TILE, readyForPark.getY(), Math.toRadians(0));
	Vector3D Test = new Vector3D(24,24, Math.toRadians(0));

	@Override
	public void setStartingPosition() {
		robot.odometry.setPositionEstimate(start);
	}

	@Override
	public void setVisionSettings() {

	}

	@Override
	public void addActions() {

		//Deposit pre-load
		actions.add(new MutlipleAction(new action[]{new DriveToPosition(robot,depositPosition1), new GoToHighDeposit(robot), new DeployIntake(robot)}));
		actions.add(new DepositFreight(robot));
		actions.add(new Delay(300));

		//Against wall lineup for first warehouse cycle + deploy intake
		actions.add(new MutlipleAction(new action[]{new DriveToPosition(robot,readyForCollection1) , new GoToInState(robot)}));

		//Intake first freight
		actions.add(new DriveToIntake(robot, collect1, 3.5, false));

		//Exit warehouse
		actions.add(new DriveToPosition(robot, readyForCollection1));

//------------------------------------------------------------------------------------------------\\

		//Deposit 1st cube
		actions.add(new MutlipleAction(new action[]{new DriveToPosition(robot,depositPosition1) , new GoToHighDeposit(robot)}));
		actions.add(new DepositFreight(robot));
		actions.add(new Delay(300));

		//Against wall lineup for first warehouse cycle + deploy intake
		actions.add(new MutlipleAction(new action[]{new DriveToPosition(robot,readyForCollection1,2.5,false) , new GoToInState(robot)}));

		//Intake first freight
		actions.add(new DriveToIntake(robot, collect2, 3.5, false));

		//Exit warehouse
		actions.add(new DriveToPosition(robot, readyForCollection1));

//------------------------------------------------------------------------------------------------\\

		//Deposit 1st cube
		actions.add(new MutlipleAction(new action[]{new DriveToPosition(robot,depositPosition1) , new GoToHighDeposit(robot)}));
		actions.add(new DepositFreight(robot));
		actions.add(new Delay(300));

		//Against wall lineup for first warehouse cycle + deploy intake
		actions.add(new MutlipleAction(new action[]{new DriveToPosition(robot,readyForCollection1,2.5,false) , new GoToInState(robot)}));

		//Intake first freight
		actions.add(new DriveToIntake(robot, collect3, 3.5, false));

		//Exit warehouse
		actions.add(new DriveToPosition(robot, readyForCollection1));

	}
}
