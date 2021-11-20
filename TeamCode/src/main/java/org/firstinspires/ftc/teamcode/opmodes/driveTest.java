package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.baseopmode.baseAuto;
import org.firstinspires.ftc.teamcode.stateMachine.actions.basedDrive;
import org.firstinspires.ftc.teamcode.stateMachine.actions.basedTurn;


@Autonomous
public class driveTest extends baseAuto {

    @Override
    public void setStartingPosition() {

    }

    @Override
    public void addActions() {

        double distance = 50;
        actions.add(new basedDrive(robot, distance));
        actions.add(new basedDrive(robot, -distance));


    }

}
