package org.firstinspires.ftc.teamcode.templateOpModes.systemIdentification;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.classicalControl.velocityControl;
import org.firstinspires.ftc.teamcode.commandBase.action;
import org.firstinspires.ftc.teamcode.commandBase.Scheduler;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

import java.util.ArrayList;

@Autonomous
public class MotorVeloTuner extends LinearOpMode {


	Robot robot = new Robot();

	Scheduler scheduler;
	velocityControl veloController;

	@Override
	public void runOpMode() {
		robot.init(hardwareMap);
		veloController = new velocityControl(robot.driveTrain.leftMotorSys);
		ArrayList<action> actions = new ArrayList<>();

		scheduler = new Scheduler(hardwareMap, actions, robot.getSubsystems());
		waitForStart();
		while(opModeIsActive()) {
			veloController.controlMotor(93.3333333333 * 5);
			scheduler.updateRobot();
		}

	}
}
