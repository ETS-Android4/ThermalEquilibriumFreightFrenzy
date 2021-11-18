package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.baseopmode.baseAuto;
import org.firstinspires.ftc.teamcode.geometry.Vector3D;
import org.firstinspires.ftc.teamcode.stateMachine.actions.aimAtPoint;
import org.firstinspires.ftc.teamcode.stateMachine.actions.basedDrive;
import org.firstinspires.ftc.teamcode.stateMachine.actions.basedTurn;

@Autonomous
public class RedDuckAuto extends baseAuto {
    public Vector3D startPosition = new Vector3D(-39, -56, Math.toRadians(-90));
    public Vector3D goalPosition = new Vector3D(-12, -24, 0);
    public Vector3D carousel = new Vector3D(-72,-72 - 24,0);
    public Vector3D park = new Vector3D(-60,-35,0);
    @Override
    public void setStartingPosition() {
        robot.setRobotPose(startPosition);
    }

    @Override
    public void addActions() {
        actions.add(new basedDrive(robot,goalPosition,-0.8));
        actions.add(new aimAtPoint(robot,goalPosition,false, true));
        actions.add(new aimAtPoint(robot,carousel,false,false));
        actions.add(new basedDrive(robot,carousel,0.48));
        actions.add(new aimAtPoint(robot, park, false, true));
        actions.add(new basedDrive(robot,park,-1.15));
        actions.add(new basedTurn(robot,Math.toRadians(-90)));
        actions.add(new basedDrive(robot,-4));
    }
}
