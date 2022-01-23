package org.firstinspires.ftc.teamcode.opmodes.competetionOpModes;

import android.service.quicksettings.Tile;

import com.acmerobotics.roadrunner.drive.Drive;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Geometry.Vector3D;
import org.firstinspires.ftc.teamcode.commandBase.autoActions.DrivetrainControl.DriveToPosition;
import org.firstinspires.ftc.teamcode.commandBase.autoActions.Misc.Delay;
import org.firstinspires.ftc.teamcode.commandBase.autoActions.Misc.OffsetOdom;
import org.firstinspires.ftc.teamcode.templateOpModes.BaseAuto;
import org.opencv.core.Mat;

import java.util.Vector;

@Autonomous
public class RedCycleAuto extends BaseAuto {


    Vector3D depositPosition = new Vector3D(-2,-TILE * 2.55 + 12,Math.toRadians(-70));
    Vector3D start = new Vector3D(TILE / 2.0, -TILE * 3 + 8.375, Math.toRadians(-90));
    Vector3D readyForCollection1 = new Vector3D(TILE / 3, -TILE * 3 + 6, Math.toRadians(0));
    Vector3D readyForCollection2 = new Vector3D(TILE / 3, -TILE * 3 + 9, Math.toRadians(0));
    Vector3D readyForCollection3 = new Vector3D(TILE / 3, -TILE * 3 + 11, Math.toRadians(0));
    Vector3D collect1 = new Vector3D(TILE * 2 - 12, readyForCollection1.getY(), Math.toRadians(0));
    Vector3D collect2 = new Vector3D(TILE * 2 - 12, readyForCollection2.getY(), Math.toRadians(0));
    Vector3D collect3 = new Vector3D(TILE * 2 - 12, readyForCollection3.getY(), Math.toRadians(0));

    Vector3D gapPose = new Vector3D(TILE, readyForCollection1.getY(), Math.toRadians(0));


    @Override
    public void setStartingPosition() {
        robot.setRobotPose(start);
    }

    @Override
    public void setVisionSettings() {

    }

    @Override
    public void addActions() {

        //deposits pre load
        actions.add(new DriveToPosition(robot,depositPosition));

        //agaisnt wall drives into warehouse intakes and deposit one block
        actions.add(new DriveToPosition(robot,readyForCollection1));
        actions.add(new DriveToPosition(robot,collect1));
        actions.add(new DriveToPosition(robot,readyForCollection1, 1.5, false));
        actions.add(new DriveToPosition(robot,depositPosition));

        //agaisnt wall drives into warehouse intakes and deposit second block
        actions.add(new DriveToPosition(robot,readyForCollection2));
        actions.add(new DriveToPosition(robot,collect2));
        actions.add(new DriveToPosition(robot,readyForCollection2, 1.5,false));
        actions.add(new DriveToPosition(robot,depositPosition));

        //agaisnt wall drives into warehouse intakes and deposit third block
        actions.add(new DriveToPosition(robot,readyForCollection3));
        actions.add(new DriveToPosition(robot,collect3));
        actions.add(new DriveToPosition(robot,readyForCollection3,1.5,false));
        actions.add(new DriveToPosition(robot,depositPosition));

        //parks in warehouse
        actions.add(new DriveToPosition(robot,readyForCollection3));
        actions.add(new DriveToPosition(robot,collect3));

    }
}
