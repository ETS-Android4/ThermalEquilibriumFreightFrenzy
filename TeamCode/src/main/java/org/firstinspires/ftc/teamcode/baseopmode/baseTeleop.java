package org.firstinspires.ftc.teamcode.baseopmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.stateMachine.scheduler;
import org.firstinspires.ftc.teamcode.stateMachine.teleopAction;
import org.firstinspires.ftc.teamcode.subsystems.robot;

import java.util.ArrayList;

public class baseTeleop extends LinearOpMode {


	protected robot robot;
	protected scheduler scheduler;
	protected ArrayList<teleopAction> actions = new ArrayList<>();

	public void addActions() {

	}

	@Override
	public void runOpMode() throws InterruptedException {
		robot = new robot();
		robot.init(hardwareMap);

		addActions();
		scheduler = new scheduler(robot.getSubsystems(), actions, hardwareMap);

		waitForStart();
		while (opModeIsActive()) {
			scheduler.updateTeleop();
		}

	}
}
