package org.firstinspires.ftc.teamcode.baseopmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.geometry.Vector3D;


@Autonomous
public class driveTest extends baseAuto {
    double distance = 45;
    double distance2 = 12;


    @Override
    public void setStartingPosition() {
        robot.setRobotPose(new Vector3D());
    }

    @Override
    public void addActions() {
        collectFromBarrier();
        collectFromBarrier();
    }

    public void collectFromBarrier() {
        driveDistance(distance);
        turn(Math.toRadians(17));
        driveDistance(distance2);
        driveDistance(-distance2);

        turn(Math.toRadians(0));
        driveDistance(-distance);

    }

}
