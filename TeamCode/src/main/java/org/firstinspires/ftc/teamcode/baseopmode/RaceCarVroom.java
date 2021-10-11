package org.firstinspires.ftc.teamcode.baseopmode;

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
public class RaceCarVroom extends LinearOpMode {
	org.firstinspires.ftc.teamcode.subsystems.robot robot;
	DifferentialDriveController controller;
	dripTeleop.drivingState dtState = dripTeleop.drivingState.MANUAL;
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
			dtState = dripTeleop.drivingState.AUTO;
		}
		if (areJoysticksActive()) {
			dtState = dripTeleop.drivingState.MANUAL;
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
					robot.driveTrain.robotRelative(gamepad1.touchpad_finger_1_x - gamepad1.right_trigger, gamepad1.left_stick_x);
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
