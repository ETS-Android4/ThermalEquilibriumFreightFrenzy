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
    Vector3D readyForCollection1 = new Vector3D(TILE / 3, -TILE * 3 + 7, Math.toRadians(0));
    Vector3D collect = new Vector3D(TILE * 2 - 12, readyForCollection1.getY(), Math.toRadians(0));
    Vector3D gapPose = new Vector3D(TILE, readyForCollection1.getY() - 1.5, Math.toRadians(0));


    @Override
    public void setStartingPosition() {
        robot.setRobotPose(start);
    }

    @Override
    public void setVisionSettings() {

    }

    @Override
    public void addActions() {


        for (int i = 0; i < 3; ++i) {
            actions.add(new DriveToPosition(robot,depositPosition));
            actions.add(new DriveToPosition(robot,readyForCollection1,1.5,false));

            actions.add(new DriveToPosition(robot,gapPose,1.5,true));
            actions.add(new DriveToPosition(robot,collect));
            actions.add(new DriveToPosition(robot,gapPose,1.5,true));
            actions.add(new DriveToPosition(robot,readyForCollection1,1.5,false));
        }
        actions.add(new DriveToPosition(robot,collect));







    }
}
