package org.firstinspires.ftc.teamcode.TemplateOpModes.systemIdentification;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.classicalControl.velocityControl;
import org.firstinspires.ftc.teamcode.CommandBase.action;
import org.firstinspires.ftc.teamcode.CommandBase.scheduler;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

import java.util.ArrayList;

@Autonomous
public class MotorVeloTuner extends LinearOpMode {


	Robot robot = new Robot();

	scheduler scheduler;
	velocityControl veloController;

	@Override
	public void runOpMode() {
		robot.init(hardwareMap);
		veloController = new velocityControl(robot.driveTrain.leftMotorSys);
		ArrayList<action> actions = new ArrayList<>();

		scheduler = new scheduler(hardwareMap, actions, robot.getSubsystems());
		waitForStart();
		while(opModeIsActive()) {
			veloController.controlMotor(93.3333333333 * 5);
			scheduler.updateRobot();
		}

	}
}
