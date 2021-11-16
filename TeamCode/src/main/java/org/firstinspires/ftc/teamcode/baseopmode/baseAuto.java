package org.firstinspires.ftc.teamcode.baseopmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.geometry.Vector3D;
import org.firstinspires.ftc.teamcode.stateMachine.action;
import org.firstinspires.ftc.teamcode.stateMachine.scheduler;
import org.firstinspires.ftc.teamcode.subsystems.robot;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.subsystems.robot.isCompBot;

public abstract class baseAuto extends LinearOpMode {

	protected robot robot;

	protected ArrayList<action> actions = new ArrayList<>();
	protected Vector3D startingPosition = new Vector3D();


	public abstract void setStartingPosition();


	public abstract void addActions();

	@Override
	public void runOpMode() {
		robot = new robot();
		if (isCompBot) {
			robot.init(hardwareMap);
		} else {
			robot.initMinimal(hardwareMap);
		}

		addActions();
		setStartingPosition();

		scheduler scheduler = new scheduler(hardwareMap, actions, robot.getSubsystems());

		waitForStart();

		while (opModeIsActive()) {
			scheduler.updateStateMachineAndRobot();
		}

	}


}
