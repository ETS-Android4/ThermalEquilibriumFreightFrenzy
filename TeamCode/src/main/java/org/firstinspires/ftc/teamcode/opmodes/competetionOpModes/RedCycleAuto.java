package org.firstinspires.ftc.teamcode.opmodes.competetionOpModes;

import com.acmerobotics.roadrunner.drive.Drive;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Geometry.Vector3D;
import org.firstinspires.ftc.teamcode.commandBase.autoActions.DrivetrainControl.DriveToPosition;
import org.firstinspires.ftc.teamcode.templateOpModes.BaseAuto;

@Autonomous
public class RedCycleAuto extends BaseAuto {


    Vector3D depositPosition = new Vector3D(0,-TILE * 2.55 + 8.375,Math.toRadians(-60));
    Vector3D start = new Vector3D(TILE / 2.0, -TILE * 3 + 8.375, Math.toRadians(-90));
    Vector3D readyForCollection1 = new Vector3D(TILE / 3, -TILE * 3 + 6.375, Math.toRadians(0));

    @Override
    public void setStartingPosition() {
        robot.setRobotPose(start);
    }

    @Override
    public void setVisionSettings() {

    }

    @Override
    public void addActions() {
        actions.add(new DriveToPosition(robot,depositPosition));
        actions.add(new DriveToPosition(robot,readyForCollection1));
    }
}
