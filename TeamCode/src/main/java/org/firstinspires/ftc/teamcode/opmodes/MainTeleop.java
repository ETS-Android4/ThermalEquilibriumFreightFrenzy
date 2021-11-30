package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.TemplateOpModes.BaseTeleop;
import org.firstinspires.ftc.teamcode.CommandBase.teleopActions.arcadeDrive;
import org.firstinspires.ftc.teamcode.CommandBase.teleopActions.commandDeposit;
import org.firstinspires.ftc.teamcode.CommandBase.teleopActions.goToCollection;
import org.firstinspires.ftc.teamcode.CommandBase.teleopActions.toggleBox;
import org.firstinspires.ftc.teamcode.CommandBase.teleopActions.toggleIntake;


@TeleOp
public class MainTeleop extends BaseTeleop {
	@Override
	public void addActions() {
		actions.add(new arcadeDrive(robot, gamepad1, gamepad2));
		actions.add(new toggleIntake(robot, gamepad1, gamepad2));
		actions.add(new commandDeposit(robot, gamepad1, gamepad2));
		actions.add(new goToCollection(robot, gamepad1, gamepad2));
		actions.add(new toggleBox(robot, gamepad1, gamepad2));
	}
}


