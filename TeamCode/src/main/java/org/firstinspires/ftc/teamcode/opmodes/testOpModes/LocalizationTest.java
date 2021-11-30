package org.firstinspires.ftc.teamcode.opmodes.testOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.commandBase.Scheduler;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

import java.util.ArrayList;

@Autonomous
public class LocalizationTest extends LinearOpMode {

    Robot robot = new Robot();

    @Override
    public void runOpMode() {
        robot.initMinimal(hardwareMap);


        Scheduler scheduler = new Scheduler(hardwareMap, new ArrayList<>(), robot.getSubsystems());


        telemetry.addData("ready!", "");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            scheduler.updateRobot();

        }

    }
}
