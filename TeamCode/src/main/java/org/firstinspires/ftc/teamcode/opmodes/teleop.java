package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.baseopmode.baseTeleop;
import org.firstinspires.ftc.teamcode.stateMachine.teleopActions.arcadeDrive;


@TeleOp
public class teleop extends baseTeleop {
	@Override
	public void addActions() {
		actions.add(new arcadeDrive(robot, gamepad1, gamepad2));
	}
}


