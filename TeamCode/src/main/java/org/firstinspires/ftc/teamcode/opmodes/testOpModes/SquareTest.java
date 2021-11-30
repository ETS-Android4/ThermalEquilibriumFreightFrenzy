package org.firstinspires.ftc.teamcode.opmodes.testOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.templateOpModes.BaseAuto;
import org.firstinspires.ftc.teamcode.commandBase.autoActions.Drive;
import org.firstinspires.ftc.teamcode.commandBase.autoActions.Turn;


@Autonomous
public class SquareTest extends BaseAuto {

    @Override
    public void setStartingPosition() {

    }

    @Override
    public void addActions() {

        double distance = 50;
        actions.add(new Drive(robot, distance));
        actions.add(new Turn(robot, Math.toRadians(90)));
        actions.add(new Drive(robot, distance));
        actions.add(new Turn(robot, Math.toRadians(180)));
        actions.add(new Drive(robot, distance));
        actions.add(new Turn(robot, Math.toRadians(270)));
        actions.add(new Drive(robot, distance));
        actions.add(new Turn(robot, Math.toRadians(0)));




    }

}
