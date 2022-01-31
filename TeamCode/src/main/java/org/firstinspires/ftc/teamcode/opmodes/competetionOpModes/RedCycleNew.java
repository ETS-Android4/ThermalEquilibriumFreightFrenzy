package org.firstinspires.ftc.teamcode.opmodes.competetionOpModes;

import android.service.quicksettings.Tile;

import com.acmerobotics.roadrunner.drive.Drive;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.apache.commons.math3.geometry.euclidean.twod.Line;
import org.firstinspires.ftc.teamcode.Geometry.Vector3D;
import org.firstinspires.ftc.teamcode.commandBase.action;
import org.firstinspires.ftc.teamcode.commandBase.autoActions.DrivetrainControl.DriveToIntake;
import org.firstinspires.ftc.teamcode.commandBase.autoActions.DrivetrainControl.DriveToPosition;
import org.firstinspires.ftc.teamcode.commandBase.autoActions.DrivetrainControl.LastResort.TimeBasedMove;
import org.firstinspires.ftc.teamcode.commandBase.autoActions.DrivetrainControl.Turn;
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

	Vector3D depositPosition1 = new Vector3D(+ 2,-TILE * 2 + 4 ,Math.toRadians(-65));

	Vector3D readyForCollection1 = new Vector3D(TILE - 15, -TILE * 3 + 6,  Math.toRadians(0));
	Vector3D readyForCollection2 = new Vector3D(TILE - 15, -TILE * 3 + 6.5,  Math.toRadians(0));
	Vector3D readyForCollection3 = new Vector3D(TILE - 15, -TILE * 3 + 6.5,  Math.toRadians(0));

	Vector3D collect1 = new Vector3D(TILE * 2 - 5,-TILE * 3 + 5.25, Math.toRadians(0));
	Vector3D collect2 = new Vector3D(TILE * 2 - 3,-TILE * 3 + 5.5, Math.toRadians(0));
	Vector3D collect3 = new Vector3D(TILE * 2 - 2,-TILE * 3 + 5.69, Math.toRadians(0));

	Vector3D readyForPark = new Vector3D(TILE / 3, -TILE * 3 + 11, Math.toRadians(0));

	Vector3D gapPose = new Vector3D(TILE, readyForPark.getY(), Math.toRadians(0));
	Vector3D Test = new Vector3D(TILE / 2.0,-24, Math.toRadians(-90));

	Vector3D LineUp = new Vector3D(TILE - 12, -TILE * 3 + 10,  Math.toRadians(0));
	Vector3D StrafeIntoWall = new Vector3D(0,.5,0);
	Vector3D newcollect1 = new Vector3D(TILE * 2 - 2,-TILE * 3 + 5.5, Math.toRadians(0));
	Vector3D newcollect2 = new Vector3D(TILE * 2 ,-TILE * 3 + 5.5, Math.toRadians(0));
	Vector3D newcollect3 = new Vector3D(TILE * 2 ,-TILE * 3 + 5.5, Math.toRadians(0));

	Vector3D NewReadyForDepo1 = new Vector3D(TILE - 15, -TILE * 3 + 5.,  Math.toRadians(0));
	Vector3D NewReadyForDepo2 = new Vector3D(TILE - 15, -TILE * 3 + 5.25,  Math.toRadians(0));
	Vector3D NewReadyForDepo3 = new Vector3D(TILE - 15, -TILE * 3 + 5.5,  Math.toRadians(0));



	protected double turn_cutoff_time_seconds = 0.7;

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
		//agaisnt wall
		actions.add(new MutlipleAction(new action[]{new DriveToPosition(robot, LineUp), new GoToInState(robot)}));
		actions.add(new TimeBasedMove(robot,StrafeIntoWall,.75));
		//Intake
		actions.add(new DriveToIntake(robot,newcollect1,3,false));
		//out of depo
		actions.add(new MutlipleAction(new action[]{new DriveToPosition(robot, NewReadyForDepo1), new TurnOnIntake(robot,false), new Delay(250)}));
		actions.add(new TurnOffIntake(robot));
//------------------------------------------------------------------------------------------------\\
		//Deposit pre-load
		actions.add(new MutlipleAction(new action[]{new DriveToPosition(robot,depositPosition1), new GoToHighDeposit(robot), new DeployIntake(robot), new Delay(250)}));
		actions.add(new DepositFreight(robot));
		actions.add(new Delay(300));
		//agaisnt wall WITH TIME BASED CODE
		actions.add(new MutlipleAction(new action[]{new DriveToPosition(robot, LineUp), new GoToInState(robot)}));
		actions.add(new TimeBasedMove(robot,StrafeIntoWall,.75));
		//Intake
		actions.add(new DriveToIntake(robot,newcollect2,3,false));
		//out of depo
		actions.add(new MutlipleAction(new action[]{new DriveToPosition(robot, NewReadyForDepo2), new TurnOnIntake(robot,false)}));
		actions.add(new TurnOffIntake(robot));
//------------------------------------------------------------------------------------------------\\
		//Deposit pre-load
		actions.add(new MutlipleAction(new action[]{new DriveToPosition(robot,depositPosition1), new GoToHighDeposit(robot), new DeployIntake(robot), new Delay(250)}));
		actions.add(new DepositFreight(robot));
		actions.add(new Delay(300));

		actions.add(new MutlipleAction(new action[]{new DriveToPosition(robot, LineUp), new GoToInState(robot)}));
		actions.add(new TimeBasedMove(robot,StrafeIntoWall,.75));

		actions.add(new DriveToIntake(robot,newcollect3,3,false));

		actions.add(new MutlipleAction(new action[]{new DriveToPosition(robot, NewReadyForDepo3), new TurnOnIntake(robot,false)}));
		actions.add(new TurnOffIntake(robot));

		/*
		//Against wall lineup for first warehouse cycle + deploy intake
		actions.add(new MutlipleAction(new action[]{new DriveToPosition(robot,readyForCollection1) , new GoToInState(robot)}));

		//Intake first freight
		actions.add(new Turn(robot,0,turn_cutoff_time_seconds));
		actions.add(new DriveToIntake(robot, collect1, 3.5, false));

		//Exit warehouse
		actions.add(new MutlipleAction(new action[]{new DriveToPosition(robot,readyForCollection1,2,false) , new TurnOnIntake(robot,false), new Delay(250)}));
		actions.add(new TurnOffIntake(robot));
		//actions.add(new DriveToPosition(robot, readyForCollection1,1.5,false));

//------------------------------------------------------------------------------------------------\\

		//Deposit 1st cubeadb connect 192.168.43.1:5555
		actions.add(new MutlipleAction(new action[]{new DriveToPosition(robot,depositPosition1) , new GoToHighDeposit(robot)}));
		actions.add(new DepositFreight(robot));
		actions.add(new Delay(300));

		//Against wall lineup for first warehouse cycle + deploy intake
		actions.add(new MutlipleAction(new action[]{new DriveToPosition(robot,readyForCollection2) , new GoToInState(robot)}));

		//Intake first freight
		//actions.add(new MutlipleAction(new action[]{new DriveToPosition(robot,collect2,2.5,false) , new TurnOnIntake(robot,true)}));
		//actions.add(new TurnOffIntake(robot));
		actions.add(new Turn(robot,0, turn_cutoff_time_seconds));
		actions.add(new DriveToIntake(robot, collect2, 3.5, false));

		//Exit warehouse
		actions.add(new MutlipleAction(new action[]{new DriveToPosition(robot,readyForCollection2,2,false) , new TurnOnIntake(robot,false), new Delay(250)}));
		actions.add(new TurnOffIntake(robot));

//------------------------------------------------------------------------------------------------\\

		//Deposit 2nd cube
		actions.add(new MutlipleAction(new action[]{new DriveToPosition(robot,depositPosition1) , new GoToHighDeposit(robot)}));
		actions.add(new DepositFreight(robot));
		actions.add(new Delay(300));

		//Against wall lineup for first warehouse cycle + deploy intake
		actions.add(new MutlipleAction(new action[]{new DriveToPosition(robot,readyForCollection3) , new GoToInState(robot)}));

		//Intake first freight
		//actions.add(new MutlipleAction(new action[]{new DriveToPosition(robot,collect3,2.5,false) , new TurnOnIntake(robot,true)}));
		//actions.add(new TurnOffIntake(robot));
		actions.add(new Turn(robot,0, turn_cutoff_time_seconds));
		actions.add(new DriveToIntake(robot, collect3, 3.5, false));

		//Exit warehouse
*/
	}
}
