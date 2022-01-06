package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.geometry.Vector3D;
import org.firstinspires.ftc.teamcode.subsystems.scoringMechanisms.Bucket;
import org.firstinspires.ftc.teamcode.subsystems.scoringMechanisms.CapArm;
import org.firstinspires.ftc.teamcode.subsystems.scoringMechanisms.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.scoringMechanisms.Intake;
import org.firstinspires.ftc.teamcode.subsystems.scoringMechanisms.TapeTurret;

import java.util.ArrayList;

public class Robot {

    private final ArrayList<subsystem> subsystems = new ArrayList<>();


    public final static boolean isCompBot = false;

    public VoltageSensor batterVoltageSensor;

    public Intake Intake = new Intake();

    public Deposit Deposit = new Deposit();

    public Drivetrain driveTrain = new Drivetrain(batterVoltageSensor);

    public Bucket bucketSys = new Bucket();

    public Dashboard dashBoard = new Dashboard();

    public DuckDetection duckDetection = new DuckDetection(dashBoard);

    public DuckWheel duckwheel = new DuckWheel();
    public DifferentialDriveOdometry odometry = new DifferentialDriveOdometry();
    public CapArm capArm = new CapArm();



    public Robot() {



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
        subsystems.add(driveTrain);
        subsystems.add(dashBoard);
        subsystems.add(odometry);
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
        duckDetection.init(hwmap);
        duckwheel.init(hwmap);
        capArm.init(hwmap);
        subsystems.add(bucketSys);
        subsystems.add(Intake);
        subsystems.add(Deposit);
        subsystems.add(duckDetection);
        subsystems.add(duckwheel);
        subsystems.add(capArm);
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
        capArm.initNoReset(hwmap);
        subsystems.add(Intake);
        subsystems.add(Deposit);
        subsystems.add(driveTrain);
        subsystems.add(dashBoard);
        subsystems.add(odometry);
        subsystems.add(capArm);
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
