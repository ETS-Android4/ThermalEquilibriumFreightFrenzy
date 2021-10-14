package org.firstinspires.ftc.teamcode.baseopmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.stateMachine.actions.basedDrive;
import org.firstinspires.ftc.teamcode.stateMachine.actions.basedTurn;


@Autonomous
public class driveTest extends baseAuto {

    @Override
    public void addActions() {

        double distance = 30;
        //actions.add(new basedTurn(robot,Math.toRadians(90)));
        actions.add(new basedDrive(robot, distance));
        actions.add(new basedTurn(robot, Math.toRadians(90)));
        actions.add(new basedDrive(robot, distance));
        actions.add(new basedTurn(robot, Math.toRadians(180)));
        actions.add(new basedDrive(robot, distance));
        actions.add(new basedTurn(robot, Math.toRadians(270)));
        actions.add(new basedDrive(robot, distance));
        actions.add(new basedTurn(robot, Math.toRadians(0)));


    }

}
