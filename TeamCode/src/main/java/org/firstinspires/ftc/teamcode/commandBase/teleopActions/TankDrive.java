package org.firstinspires.ftc.teamcode.commandBase.teleopActions;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystems.Robot;

@Deprecated
public class TankDrive extends ArcadeDrive {
	public TankDrive(Robot robot, Gamepad gamepad1, Gamepad gamepad2) {
		super(robot, gamepad1, gamepad2);
	}

	@Override
	public void periodic() {
		robot.driveTrain.setMotorPowers(-gamepad1.left_stick_y, -gamepad1.right_stick_y);
	}

}
