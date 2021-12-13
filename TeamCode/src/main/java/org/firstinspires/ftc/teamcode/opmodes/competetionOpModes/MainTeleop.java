package org.firstinspires.ftc.teamcode.opmodes.competetionOpModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commandBase.teleopActions.ToggleDuckWheel;
import org.firstinspires.ftc.teamcode.templateOpModes.BaseTeleop;
import org.firstinspires.ftc.teamcode.commandBase.teleopActions.ArcadeDrive;
import org.firstinspires.ftc.teamcode.commandBase.teleopActions.CommandDeposit;
import org.firstinspires.ftc.teamcode.commandBase.teleopActions.GoToCollectionState;
import org.firstinspires.ftc.teamcode.commandBase.teleopActions.ToggleBox;
import org.firstinspires.ftc.teamcode.commandBase.teleopActions.ToggleIntake;


@TeleOp
public class MainTeleop extends BaseTeleop {
	@Override
	public void addActions() {
		actions.add(new ArcadeDrive(robot, gamepad1, gamepad2));
		actions.add(new ToggleIntake(robot, gamepad1, gamepad2));
		actions.add(new CommandDeposit(robot, gamepad1, gamepad2));
		actions.add(new GoToCollectionState(robot, gamepad1, gamepad2));
		actions.add(new ToggleBox(robot, gamepad1, gamepad2));
		actions.add(new ToggleDuckWheel(robot,gamepad1,gamepad2));
	}
}


