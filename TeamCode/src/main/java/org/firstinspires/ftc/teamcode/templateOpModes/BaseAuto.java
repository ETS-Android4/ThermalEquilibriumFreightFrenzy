package org.firstinspires.ftc.teamcode.templateOpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.geometry.Vector3D;
import org.firstinspires.ftc.teamcode.commandBase.action;
import org.firstinspires.ftc.teamcode.commandBase.Scheduler;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.subsystems.Robot.isCompBot;

public abstract class BaseAuto extends LinearOpMode {

	protected Robot robot;

	protected ArrayList<action> actions = new ArrayList<>();
	protected Vector3D startingPosition = new Vector3D();


	public abstract void setStartingPosition();


	public abstract void addActions();

	@Override
	public void runOpMode() {
		robot = new Robot();
		if (isCompBot) {
			robot.init(hardwareMap);
		} else {
			robot.initMinimal(hardwareMap);
		}

		addActions();
		setStartingPosition();

		Scheduler scheduler = new Scheduler(hardwareMap, actions, robot.getSubsystems());

		waitForStart();

		while (opModeIsActive()) {
			scheduler.updateStateMachineAndRobot();
		}

	}


}
