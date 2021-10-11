package org.firstinspires.ftc.teamcode.baseopmode.systemIdentification;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.classicalControl.velocityControl;
import org.firstinspires.ftc.teamcode.stateMachine.action;
import org.firstinspires.ftc.teamcode.stateMachine.scheduler;
import org.firstinspires.ftc.teamcode.subsystems.robot;

import java.util.ArrayList;

@Autonomous
public class motorVeloTuner extends LinearOpMode {


	robot robot = new robot();

	scheduler scheduler;
	velocityControl veloController;

	@Override
	public void runOpMode() throws InterruptedException {
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
