package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.CommandBase.scheduler;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

import java.util.ArrayList;

@Autonomous
public class LocalizationTest extends LinearOpMode {

    Robot robot = new Robot();

    @Override
    public void runOpMode() {
        robot.initMinimal(hardwareMap);


        scheduler scheduler = new scheduler(hardwareMap, new ArrayList<>(), robot.getSubsystems());


        telemetry.addData("ready!", "");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            scheduler.updateRobot();

        }

    }
}
