package org.firstinspires.ftc.teamcode.baseopmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.geometry.Vector3D;


@Autonomous
public class driveTest extends baseAuto {

    @Override
    public void addActions() {
        safeDrive(new Vector3D(30,0,0));
        safeDrive(new Vector3D(20,20,90));
        safeDrive(new Vector3D(0,0,0));
    }

}
