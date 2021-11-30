package org.firstinspires.ftc.teamcode.opmodes.competetionOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.templateOpModes.BaseAuto;
import org.firstinspires.ftc.teamcode.geometry.Vector3D;
import org.firstinspires.ftc.teamcode.commandBase.autoActions.AimAtPoint;
import org.firstinspires.ftc.teamcode.commandBase.autoActions.Drive;
import org.firstinspires.ftc.teamcode.commandBase.autoActions.Turn;

@Autonomous
public class RedDuckAuto extends BaseAuto {
    public Vector3D startPosition = new Vector3D(-39, -56, Math.toRadians(-90));
    public Vector3D goalPosition = new Vector3D(-12, -24, 0);
    public Vector3D carousel = new Vector3D(-72,-72,0);
    public Vector3D park = new Vector3D(-60,-35,0);
    @Override
    public void setStartingPosition() {
        robot.setRobotPose(startPosition);
    }

    @Override
    public void addActions() {
//        // drive to goal to deposit
//        actions.add(new basedDrive(robot,goalPosition,-0.6));
//        actions.add(new aimAtPoint(robot,goalPosition,false, true));
//
//        // drive to carousel
//        actions.add(new aimAtPoint(robot,carousel,false,false));
//        actions.add(new basedDrive(robot,carousel,0.5));
//
//        // collect duck
//        actions.add(new basedTurn(robot, startPosition.getAngleRadians()));
//
//        // deposit the duck
//        actions.add(new basedDrive(robot,goalPosition,-0.6));
//        actions.add(new aimAtPoint(robot,goalPosition,false, true));
//
//        // go to park
//        actions.add(new aimAtPoint(robot, park, false, true));
//        actions.add(new basedDrive(robot,park,-1));
//
//        // make sure we are parked
//        actions.add(new basedTurn(robot,Math.toRadians(-90)));
//        actions.add(new basedDrive(robot,-5));

        // drive to goal to deposit
        actions.add(new Drive(robot,-5));
        actions.add(new Drive(robot,goalPosition,-1, -18));
        actions.add(new AimAtPoint(robot,goalPosition,false, true));

        // drive to carousel
        actions.add(new AimAtPoint(robot,carousel,false,false));
        actions.add(new Drive(robot,carousel,1,-24));

        // collect duck
        actions.add(new Turn(robot, startPosition.getAngleRadians()));
        actions.add(new Drive(robot, 3));
        actions.add(new Drive(robot,-3));

        // deposit the duck
        actions.add(new Drive(robot,goalPosition,-1, -18));
        actions.add(new AimAtPoint(robot,goalPosition,false, true));

        // go to park
        actions.add(new AimAtPoint(robot, park, false, true));
        actions.add(new Drive(robot,park,-1));

        // make sure we are parked
        actions.add(new Turn(robot,Math.toRadians(-90)));
        actions.add(new Drive(robot,-5));
    }
}
