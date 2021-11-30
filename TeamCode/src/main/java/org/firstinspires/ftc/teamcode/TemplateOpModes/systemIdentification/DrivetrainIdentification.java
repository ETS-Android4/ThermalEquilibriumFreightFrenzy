package org.firstinspires.ftc.teamcode.TemplateOpModes.systemIdentification;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Robot;

@TeleOp
public class DrivetrainIdentification extends LinearOpMode {

    Robot robot = new Robot();

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

            if (rampTimer.seconds() > 0.5) {
                power = 1;
            }

            //robot.driveTrain.robotRelative(power, 0);
            robot.driveTrain.leftMotorSys.input(power);
            robot.driveTrain.rightMotorSys.input(power);
            robot.driveTrain.update();
            robot.dashBoard.update();

            System.out.println("step response: " + power + ", " + robot.driveTrain.rightMotorSys.getState().getPosition() + ", " + robot.driveTrain.leftMotorSys.getState().getVelocity());

        }


    }

}
