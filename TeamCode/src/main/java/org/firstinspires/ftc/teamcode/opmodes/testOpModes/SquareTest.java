package org.firstinspires.ftc.teamcode.opmodes.testOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.templateOpModes.BaseAuto;
import org.firstinspires.ftc.teamcode.commandBase.autoActions.DrivetrainControl.Drive;
import org.firstinspires.ftc.teamcode.commandBase.autoActions.DrivetrainControl.Turn;


@Disabled
public class SquareTest extends BaseAuto {

    @Override
    public void setStartingPosition() {

    }

    @Override
    public void setVisionSettings() {

    }

    @Override
    public void addActions() {

        double distance = 20;
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
