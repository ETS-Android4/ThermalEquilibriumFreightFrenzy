package org.firstinspires.ftc.teamcode.TemplateOpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.CommandBase.scheduler;
import org.firstinspires.ftc.teamcode.CommandBase.teleopAction;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

import java.util.ArrayList;

public class BaseTeleop extends LinearOpMode {


	protected Robot robot;
	protected scheduler scheduler;
	protected ArrayList<teleopAction> actions = new ArrayList<>();

	public void addActions() {

	}

	@Override
	public void runOpMode() {
		robot = new Robot();
		robot.init(hardwareMap);

		addActions();
		scheduler = new scheduler(robot.getSubsystems(), actions, hardwareMap);

		waitForStart();
		while (opModeIsActive()) {
			scheduler.updateTeleop();
		}

	}
}
