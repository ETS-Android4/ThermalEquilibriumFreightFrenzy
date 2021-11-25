package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.geometry.Vector3D;
import org.firstinspires.ftc.teamcode.subsystems.scoringMechanisms.bucket;
import org.firstinspires.ftc.teamcode.subsystems.scoringMechanisms.deposit;
import org.firstinspires.ftc.teamcode.subsystems.scoringMechanisms.intake;

import java.util.ArrayList;

public class robot {

    private final ArrayList<subsystem> subsystems = new ArrayList<>();


    public final static boolean isCompBot = false;

    public VoltageSensor batterVoltageSensor;

    public intake Intake = new intake();

    public deposit Deposit = new deposit();

    public drivetrain driveTrain = new drivetrain(batterVoltageSensor);

    public bucket bucketSys = new bucket();

    public dashboard dashBoard = new dashboard();

    public DuckDetection duckDetection = new DuckDetection(dashBoard);



    public differentialDriveOdom odometry = new differentialDriveOdom();


    /**
     * full robot constructor, use for competition deployment
     */
    public robot() {



    }


    /**
     * initialize only drive train and dashboard subsystems
     *
     * @param hwmap HardwareMap instance
     */
    public void initMinimal(HardwareMap hwmap) {
        batterVoltageSensor = hwmap.voltageSensor.iterator().next();
        driveTrain.init(hwmap);
        dashBoard.init(hwmap);
        odometry.init(hwmap);
        duckDetection.init(hwmap);
        subsystems.add(driveTrain);
        subsystems.add(dashBoard);
        subsystems.add(odometry);
        subsystems.add(duckDetection);
    }

    /**
     * initialization including reset of subsystems
     *
     * @param hwmap HardwareMap instance
     */
    public void init(HardwareMap hwmap) {

        initMinimal(hwmap);

        Intake.init(hwmap);
        Deposit.init(hwmap);
        bucketSys.init(hwmap);

        subsystems.add(bucketSys);
        subsystems.add(Intake);
        subsystems.add(Deposit);
    }

    /**
     * initialization but without resetting certain subsystems
     * such as the encoder of linear slides and things that need to retain position between auto and teleop
     * @param hwmap HardwareMap instance
     */
    public void initWithoutReset(HardwareMap hwmap) {
        driveTrain.initNoReset(hwmap);
        dashBoard.initNoReset(hwmap);
        odometry.initNoReset(hwmap);
        Intake.initNoReset(hwmap);
        Deposit.initNoReset(hwmap);
        subsystems.add(Intake);
        subsystems.add(Deposit);
        subsystems.add(driveTrain);
        subsystems.add(dashBoard);
        subsystems.add(odometry);
    }

    /**
     * obtain the robot position
     *
     * @return drivetrain position
     */
    public Vector3D getRobotPose() {
        return odometry.subsystemState();
    }
    public void setRobotPose(Vector3D pose) {
        odometry.setPositionEstimate(pose);
    }

    /**
     * @return array list of subsystems
     */
    public ArrayList<subsystem> getSubsystems() {
        return subsystems;
    }


    public Vector3D getVelocity() {
        return odometry.getVelocity();
    }
}
