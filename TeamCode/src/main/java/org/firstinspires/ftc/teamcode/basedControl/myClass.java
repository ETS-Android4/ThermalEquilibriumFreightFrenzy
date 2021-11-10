package org.firstinspires.ftc.teamcode.basedControl;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class myClass extends LinearOpMode {

	private DcMotorEx leftDrive;
	private DcMotorEx rightDrive;

	@Override
	public void runOpMode() throws InterruptedException {
		leftDrive = hardwareMap.get(DcMotorEx.class, "l");
		rightDrive = hardwareMap.get(DcMotorEx.class, "r");
		waitForStart();
		while (opModeIsActive()) {
			double forwardPower = -gamepad1.left_stick_y;
			double turnPower = gamepad1.right_stick_x;
			double leftCommand = forwardPower + turnPower;
			double rightCommand = forwardPower - turnPower;
			leftDrive.setPower(leftCommand);
			rightDrive.setPower(rightCommand);

		}
	}
}
