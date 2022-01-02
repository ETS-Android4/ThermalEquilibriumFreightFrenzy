package org.firstinspires.ftc.teamcode.templateOpModes.systemIdentification;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.DifferentialDriveOdometry;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@Autonomous
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
                power = 0.25;
            }
            if (rampTimer.seconds() > 2) {
                power = 0.75;
            }

            robot.driveTrain.leftMotorSys.input(power);
            robot.driveTrain.rightMotorSys.input(power);
            robot.driveTrain.update();
            robot.dashBoard.update();

            System.out.println("step response: " + power + ", " + DifferentialDriveOdometry.encoderTicksToInches(robot.driveTrain.rightMotorSys.getState().getPosition())
                    + ", " + DifferentialDriveOdometry.encoderTicksToInches(robot.driveTrain.leftMotorSys.getState().getVelocity()));

        }


    }

}
