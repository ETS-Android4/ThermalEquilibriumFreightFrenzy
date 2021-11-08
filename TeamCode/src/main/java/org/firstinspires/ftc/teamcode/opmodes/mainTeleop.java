package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.baseopmode.baseTeleop;
import org.firstinspires.ftc.teamcode.stateMachine.teleopActions.arcadeDrive;
import org.firstinspires.ftc.teamcode.stateMachine.teleopActions.commandDeposit;
import org.firstinspires.ftc.teamcode.stateMachine.teleopActions.goToCollection;
import org.firstinspires.ftc.teamcode.stateMachine.teleopActions.toggleBox;
import org.firstinspires.ftc.teamcode.stateMachine.teleopActions.toggleIntake;


@TeleOp
public class mainTeleop extends baseTeleop {
	@Override
	public void addActions() {
		actions.add(new arcadeDrive(robot, gamepad1, gamepad2));
		actions.add(new toggleIntake(robot, gamepad1, gamepad2));
		actions.add(new commandDeposit(robot, gamepad1, gamepad2));
		actions.add(new goToCollection(robot, gamepad1, gamepad2));
		actions.add(new toggleBox(robot, gamepad1, gamepad2));
	}
}


