package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.geometry.Vector3D;

import java.util.ArrayList;

public class robot {

    private ArrayList<subsystem> subsystems = new ArrayList<>();


    public VoltageSensor batterVoltageSensor;

    public drivetrain driveTrain = new drivetrain();

    public dashboard dashBoard = new dashboard();



    /**
     * full robot constructor, use for competition deployment
     */
    public robot() {

        subsystems.add(driveTrain);
        subsystems.add(dashBoard);

    }

    /**
     * alternate constructor that allows us to only use the drive train and dashboard for test robot
     *
     * @param minimal
     */
    public robot(boolean minimal) {
        subsystems.add(driveTrain);
        subsystems.add(dashBoard);
    }


    /**
     * initialize only drive train and dashboard subsystems
     *
     * @param hwmap HardwareMap instance
     */
    public void initMinimal(HardwareMap hwmap) {
        driveTrain.init(hwmap);
        dashBoard.init(hwmap);
        batterVoltageSensor = hwmap.voltageSensor.iterator().next();
    }

    /**
     * initialization including reset of subsystems
     *
     * @param hwmap HardwareMap instance
     */
    public void init(HardwareMap hwmap) {

        driveTrain.init(hwmap);
        dashBoard.init(hwmap);

        batterVoltageSensor = hwmap.voltageSensor.iterator().next();

    }

    /**
     * initialization but without resetting certain subsystems
     * such as the encoder of linear slides and things that need to retain position between auto and teleop
     * @param hwmap HardwareMap instance
     */
    public void initWithoutReset(HardwareMap hwmap) {
        driveTrain.initNoReset(hwmap);
        dashBoard.initNoReset(hwmap);
        batterVoltageSensor = hwmap.voltageSensor.iterator().next();
    }

    /**
     * obtain the robot position
     *
     * @return drivetrain position
     */
    public Vector3D getRobotPose() {
        return driveTrain.robotPosition;
    }

    /**
     * @return array list of subsystems
     */
    public ArrayList<subsystem> getSubsystems() {
        return subsystems;
    }


}
