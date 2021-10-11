package org.firstinspires.ftc.teamcode.baseopmode.systemIdentification;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.robot;

@TeleOp
public class drivetrainIdentification extends LinearOpMode {

    robot robot = new robot();

    /**
     * perform a series of step / ramp tests inorder to use for Matlab system identification
     */
    @Override
    public void runOpMode() {



        robot.initMinimal(hardwareMap);


        waitForStart();
        ElapsedTime rampTimer = new ElapsedTime();
        double power = 0;

        while (opModeIsActive()) {

            if (rampTimer.seconds() > 1)
            {
                power = 1;
            }

            //robot.driveTrain.robotRelative(power, 0);
            robot.driveTrain.leftMotorSys.input(power);
            robot.driveTrain.rightMotorSys.input(power);
            robot.driveTrain.update();
            robot.dashBoard.update();

            System.out.println("step response: " + power + ", " + robot.driveTrain.rightMotorSys.getState().getVelocity() + ", " + robot.driveTrain.leftMotorSys.getState().getVelocity());

        }


    }

}
