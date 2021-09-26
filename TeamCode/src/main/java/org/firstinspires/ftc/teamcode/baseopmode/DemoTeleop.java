package org.firstinspires.ftc.teamcode.baseopmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.robot;

@TeleOp
public class DemoTeleop extends LinearOpMode {

    org.firstinspires.ftc.teamcode.subsystems.robot robot = new robot();
    @Override
    public void runOpMode() throws InterruptedException {
        robot.initMinimal(hardwareMap);
        waitForStart();
        while(opModeIsActive()){
            double leftStickY = -gamepad1.left_stick_y;
            double leftStickX = gamepad1.left_stick_x;
            double rightStickX = gamepad1.right_stick_x;
            double flPower = leftStickY + leftStickX + rightStickX;
            double frPower = leftStickY - leftStickX - rightStickX;
            double blPower = leftStickY - leftStickX + rightStickX;
            double brPower = leftStickY + leftStickX - rightStickX;
            robot.driveTrain.FrontLeft.setPower(flPower);
            robot.driveTrain.FrontRight.setPower(frPower);
            robot.driveTrain.BackLeft.setPower(blPower);
            robot.driveTrain.BackRight.setPower(brPower);
        }
    }
}
