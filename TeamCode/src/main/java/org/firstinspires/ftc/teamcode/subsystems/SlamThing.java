package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.geometry.Vector3D;

import java.util.ArrayList;
import java.util.Vector;

import static org.firstinspires.ftc.teamcode.utils.utils.connectVectors;
import static org.firstinspires.ftc.teamcode.utils.utils.drawRobotBlue;
import static org.firstinspires.ftc.teamcode.utils.utils.visualizeVector;

public class SlamThing implements subsystem{

    public Rev2mDistanceSensor leftSensor;
    public Rev2mDistanceSensor rightSensor;
    public Rev2mDistanceSensor rearSensor;

    public Vector3D leftDistanceSensorRobotRelative = new Vector3D(0,0,Math.toRadians(-120));
    public Vector3D rearDistanceSensorRobotRelative = new Vector3D(0,0,Math.toRadians(0));
    public Vector3D rightDistanceSensorRobotRelative = new Vector3D(0,0,Math.toRadians(120));

    public Vector3D leftPosition = new Vector3D();
    public Vector3D rightPosition = new Vector3D();
    public Vector3D rearPosition = new Vector3D();

    public Vector3D previousLeft = new Vector3D();
    public Vector3D previousRight = new Vector3D();
    public Vector3D previousRear = new Vector3D();

    public Vector3D estimatedPose = new Vector3D();


    public double leftDistance = 0;
    public double rightDistance = 0;
    public double rearDistance = 0;

    DifferentialDriveOdometry odom;

    ArrayList<Vector3D> previousVectors = new ArrayList<>();

    public SlamThing(DifferentialDriveOdometry odom) {
        this.odom = odom;
    }


    @Override
    public void init(HardwareMap hwmap) {

        this.leftSensor = hwmap.get(Rev2mDistanceSensor.class, "LeftDistance");
        this.rightSensor = hwmap.get(Rev2mDistanceSensor.class, "RightDistance");
        this.rearSensor = hwmap.get(Rev2mDistanceSensor.class, "RearDistance");


    }

    @Override
    public void initNoReset(HardwareMap hwmap) {
        init(hwmap);
    }

    @Override
    public void update() {

        readSensors();

        calculatePositions();

        Dashboard.packet.put("left distance sensor", leftDistance);
        Dashboard.packet.put("right distance sensor", rightDistance);
        Dashboard.packet.put("rear distance sensor", rearDistance);

    }

    @Override
    public Object subsystemState() {
        return null;
    }

    protected void readSensors() {
        this.leftDistance = leftSensor.getDistance(DistanceUnit.INCH);
        this.rearDistance = rearSensor.getDistance(DistanceUnit.INCH);
        this.rightDistance = rightSensor.getDistance(DistanceUnit.INCH);
    }

    public void calculatePositions() {

        Vector3D robotPose = odom.subsystemState();

        double robotHeading = robotPose.getAngleRadians();
        double x = robotPose.getX();
        double y = robotPose.getY();

        double leftAngle = leftDistanceSensorRobotRelative.getAngleRadians() + robotHeading;
        double rightAngle = rightDistanceSensorRobotRelative.getAngleRadians() + robotHeading;
        double rearAngle = rearDistanceSensorRobotRelative.getAngleRadians() + robotHeading;


        double leftX = x + (leftDistance * Math.cos(leftAngle));
        double leftY = y + (leftDistance * Math.sin(leftAngle));

        double rightX = x + (rightDistance * Math.cos(rightAngle));
        double rightY = y + (rightDistance * Math.sin(rightAngle));

        double rearX = x + (rearDistance * Math.cos(rearAngle));
        double rearY = y + (rearDistance * Math.sin(rearAngle));

        double cutoffDistance = 322;

        if (leftDistance < cutoffDistance) leftPosition = new Vector3D(leftX, leftY, 0);
        if (rightDistance < cutoffDistance) rightPosition = new Vector3D(rightX,rightY,0);
        if (rearDistance < cutoffDistance) rearPosition = new Vector3D(rearX, rearY,0);

        previousVectors.add(leftPosition);
        previousVectors.add(rightPosition);
        previousVectors.add(rearPosition);


        int cutoff = 20;

        int arraylength = previousVectors.toArray().length;

        if (arraylength < cutoff) {
            for (Vector3D v : previousVectors) {
                visualizeVector(v,Dashboard.packet);
            }
        } else {
            for (int i = arraylength - cutoff; i < arraylength; ++i) {
                visualizeVector(previousVectors.get(i), Dashboard.packet);
            }
        }

        Vector3D deltaRight = rightPosition.getError(previousRight);
        Vector3D deltaLeft = leftPosition.getError(previousLeft);
        Vector3D deltaRear = rearPosition.getError(previousRear);

        double xAverage = deltaLeft.getX() + deltaRight.getX() + deltaRear.getX();
        xAverage /= 3;
        double yAverage = deltaLeft.getY() + deltaRight.getY() + deltaRear.getY();
        yAverage /= 3;

        Vector3D delta = new Vector3D(xAverage,yAverage,0);
        estimatedPose = estimatedPose.add(delta);

        drawRobotBlue(estimatedPose,Dashboard.packet);


        connectVectors(robotPose, leftPosition, Dashboard.packet);
        connectVectors(robotPose, rightPosition, Dashboard.packet);
        connectVectors(robotPose, rearPosition, Dashboard.packet);

        previousLeft = leftPosition;
        previousRight = rightPosition;
        previousRear = rearPosition;



    }
}
