package org.firstinspires.ftc.teamcode.opmodes.competetionOpModes;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commandBase.teleopActions.SafeArcadeDrive;
import org.firstinspires.ftc.teamcode.commandBase.teleopActions.ToggleDuckWheel;
import org.firstinspires.ftc.teamcode.commandBase.teleopActions.ToggleDuckWheelBetter;
import org.firstinspires.ftc.teamcode.commandBase.teleopActions.UseCap;
import org.firstinspires.ftc.teamcode.templateOpModes.BaseTeleop;
import org.firstinspires.ftc.teamcode.commandBase.teleopActions.ArcadeDrive;
import org.firstinspires.ftc.teamcode.commandBase.teleopActions.CommandDeposit;
import org.firstinspires.ftc.teamcode.commandBase.teleopActions.GoToCollectionState;
import org.firstinspires.ftc.teamcode.commandBase.teleopActions.ToggleBox;
import org.firstinspires.ftc.teamcode.commandBase.teleopActions.ToggleIntake;


@TeleOp
public class MainTeleop extends BaseTeleop {
	@RequiresApi(api = Build.VERSION_CODES.N)
	@Override
	public void addActions() {
		actions.add(new SafeArcadeDrive(robot, gamepad1, gamepad2));
		actions.add(new ToggleIntake(robot, gamepad1, gamepad2));
		actions.add(new CommandDeposit(robot, gamepad1, gamepad2));
		//actions.add(new GoToCollectionState(robot, gamepad1, gamepad2));
		actions.add(new ToggleBox(robot, gamepad1, gamepad2));
		actions.add(new ToggleDuckWheelBetter(robot,gamepad1,gamepad2));
		actions.add(new UseCap(robot,gamepad1,gamepad2));
	}
}


