package org.firstinspires.ftc.teamcode.filter;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.geometry.Vector3D;
import org.firstinspires.ftc.teamcode.roadrunnerquickstart.SampleMecanumDrive;

import static org.firstinspires.ftc.teamcode.utils.utils.AngleWrap;

public class RoadrunnerIMUEstimate {


    long counter = 0;


    double covariance = 0.9;

    SampleMecanumDrive roadrunner;

    private BNO055IMU imu;
    private BNO055IMU.Parameters IMU_Parameters;


    public RoadrunnerIMUEstimate(HardwareMap hwmap, Vector3D startPosition) {
        roadrunner = new SampleMecanumDrive(hwmap);
        roadrunner.setPoseEstimate(startPosition.toPose2d());
        imu = hwmap.get(BNO055IMU.class, "imu");
        IMU_Parameters = new BNO055IMU.Parameters();
        IMU_Parameters.mode = BNO055IMU.SensorMode.IMU;
        imu.initialize(IMU_Parameters);

        while (!imu.isGyroCalibrated() && !imu.isAccelerometerCalibrated() && !imu.isMagnetometerCalibrated()) {
            if (!imu.isGyroCalibrated()) {
                System.out.println("Gyro is not calibrated yet...");
            }
            if (!imu.isAccelerometerCalibrated()) {
                System.out.println("Accelerometer is not calibrated yet...");
            }
            if (!imu.isMagnetometerCalibrated()) {
                System.out.println("Magnetometer is not calibrated yet...");
            }
        }

    }


    public void update() {


        roadrunner.updatePoseEstimate();


        if (counter % 10 == 0) {
            Pose2d currentEstimate = roadrunner.getPoseEstimate();
            double imuAngle = imu.getAngularOrientation().firstAngle;
            System.out.println("IMU angle" + imuAngle);
            System.out.println("Roadrunner angle" + roadrunner.getPoseEstimate().getHeading());
            Pose2d poseUpdate = new Pose2d(currentEstimate.getX(), currentEstimate.getY(), AngleWrap((currentEstimate.getHeading() * (covariance)) + imuAngle * (1 - covariance)));
            roadrunner.setPoseEstimate(poseUpdate);
            counter = 0;
        }

        counter += 1;


    }

    public Vector3D getPoseEstimate() {
        return new Vector3D(roadrunner.getPoseEstimate());
    }

    public Pose2d getPoseVelocity() {
        return roadrunner.getPoseVelocity();
    }

}
