package org.firstinspires.ftc.teamcode.baseopmode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.geometry.Vector3D;
import org.firstinspires.ftc.teamcode.stateMachine.action;
import org.firstinspires.ftc.teamcode.stateMachine.actions.aimAtPoint;
import org.firstinspires.ftc.teamcode.stateMachine.actions.driveToPositionDifferential;
import org.firstinspires.ftc.teamcode.stateMachine.scheduler;
import org.firstinspires.ftc.teamcode.subsystems.robot;

import java.util.ArrayList;

@Autonomous
public class SquareTest extends LinearOpMode {

    robot robot;

    ArrayList<action> actions = new ArrayList<>();

    @Override
    public void runOpMode() {
        robot = new robot(true);
        robot.init(hardwareMap);
        actions.add(new aimAtPoint(robot,new Vector3D(20,20,0)));

        actions.add(new driveToPositionDifferential(robot,new Vector3D(20,20,Math.toRadians(0))));
        actions.add(new aimAtPoint(robot,new Vector3D(50,20,Math.toRadians(90))));

        actions.add(new driveToPositionDifferential(robot,new Vector3D(50,20,Math.toRadians(90))));
        actions.add(new aimAtPoint(robot,new Vector3D(0,0,Math.toRadians(0))));

        actions.add(new driveToPositionDifferential(robot,new Vector3D(0,0,Math.toRadians(0))));



        scheduler scheduler = new scheduler(hardwareMap, actions, robot.getSubsystems());

        waitForStart();

        while (opModeIsActive()) {


            scheduler.updateStateMachineAndRobot();
        }

    }
}
