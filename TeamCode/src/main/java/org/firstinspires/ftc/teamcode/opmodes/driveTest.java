package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.baseopmode.baseAuto;
import org.firstinspires.ftc.teamcode.geometry.Vector3D;
import org.firstinspires.ftc.teamcode.stateMachine.actions.aimAtPoint;


@Autonomous
public class driveTest extends baseAuto {

    @Override
    public void addActions() {

//        double distance = 20;
//        //actions.add(new basedTurn(robot,Math.toRadians(90)));
//        actions.add(new basedDrive(robot, distance));
//        actions.add(new basedTurn(robot, Math.toRadians(90)));
//        actions.add(new basedDrive(robot, distance));
//        actions.add(new basedTurn(robot, Math.toRadians(180)));
//        actions.add(new basedDrive(robot, distance));
//        actions.add(new basedTurn(robot, Math.toRadians(270)));
//        actions.add(new basedDrive(robot, distance));
//        actions.add(new basedTurn(robot, Math.toRadians(0)));
        actions.add(new aimAtPoint(robot, new Vector3D(-10, 0, 0)));

    }

}
