package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.classicalControl.DifferentialDriveController;
import org.firstinspires.ftc.teamcode.geometry.Vector3D;
import org.firstinspires.ftc.teamcode.humanInteraction.ButtonPress;
import org.firstinspires.ftc.teamcode.stateMachine.action;
import org.firstinspires.ftc.teamcode.stateMachine.scheduler;
import org.firstinspires.ftc.teamcode.subsystems.robot;

import java.util.ArrayList;

@TeleOp
public class dripTeleop extends LinearOpMode {

	org.firstinspires.ftc.teamcode.subsystems.robot robot;
	DifferentialDriveController controller;
	drivingState dtState = drivingState.MANUAL;
	Vector3D targetPosition = new Vector3D();
	ButtonPress a = new ButtonPress();

	@Override
	public void runOpMode() throws InterruptedException {
		robot = new robot();

		robot.initMinimal(hardwareMap);
		controller = new DifferentialDriveController(robot);
		scheduler scheduler = new scheduler(hardwareMap, new ArrayList<action>(), robot.getSubsystems());
		waitForStart();


		while (opModeIsActive()) {
			a.button(gamepad1.a);

			switchDrivingState();
			driveRobot();
			if (gamepad1.dpad_up) {
				targetPosition = robot.getRobotPose();
			}
			scheduler.updateRobot();



		}


	}

	public boolean areJoysticksActive() {
		return Math.abs(gamepad1.left_stick_x) > 0.1 || Math.abs(gamepad1.left_stick_y) > 0.1;
	}

	public void switchDrivingState() {

		if (a.press()) {
			dtState = drivingState.AUTO;
		}
		if (areJoysticksActive()) {
			dtState = drivingState.MANUAL;
		}

	}

	public void driveRobot() {
		if (gamepad1.dpad_left) {
			controller.driveToAngle(new Vector3D(0,0,Math.toRadians(-90)));
		} else if (gamepad1.dpad_right) {
			controller.driveToAngle(new Vector3D(0,0,Math.toRadians(90)));
		} else {
			switch (dtState) {
				case MANUAL:
					robot.driveTrain.robotRelative(-gamepad1.left_stick_y, gamepad1.left_stick_x);
					break;
				case AUTO:
					controller.driveToPosition(targetPosition);
					break;
			}
		}
	}

	enum drivingState {
		MANUAL,
		AUTO
	}

}
