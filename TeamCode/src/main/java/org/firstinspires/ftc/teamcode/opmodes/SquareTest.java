package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.TemplateOpModes.BaseAuto;
import org.firstinspires.ftc.teamcode.CommandBase.actions.basedDrive;
import org.firstinspires.ftc.teamcode.CommandBase.actions.basedTurn;


@Autonomous
public class SquareTest extends BaseAuto {

    @Override
    public void setStartingPosition() {

    }

    @Override
    public void addActions() {

        double distance = 50;
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
