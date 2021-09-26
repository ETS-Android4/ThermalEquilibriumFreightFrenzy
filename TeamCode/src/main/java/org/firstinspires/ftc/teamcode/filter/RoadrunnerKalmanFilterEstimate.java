package org.firstinspires.ftc.teamcode.filter;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.ejml.data.DMatrixRMaj;
import org.firstinspires.ftc.teamcode.geometry.Vector3D;
import org.firstinspires.ftc.teamcode.roadrunnerquickstart.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.utils.utils;

public class RoadrunnerKalmanFilterEstimate {

    long loop = 0;
    SampleMecanumDrive roadrunner;
    KalmanFilter estimator = new KalmanFilter();
    private BNO055IMU imu;
    private BNO055IMU.Parameters IMU_Parameters;


    public RoadrunnerKalmanFilterEstimate(HardwareMap hwmap, Vector3D startPosition) {
        roadrunner = new SampleMecanumDrive(hwmap);
        imu = hwmap.get(BNO055IMU.class, "imu");
        IMU_Parameters = new BNO055IMU.Parameters();
        IMU_Parameters.mode = BNO055IMU.SensorMode.IMU;
        imu.initialize(IMU_Parameters);

        while (!imu.isGyroCalibrated() && !imu.isAccelerometerCalibrated()) {
            if (!imu.isGyroCalibrated()) {
                System.out.println("Gyro is not calibrated yet...");
            }
            if (!imu.isAccelerometerCalibrated()) {
                System.out.println("Accelerometer is not calibrated yet...");
            }
        }

        Pose2d firstEstimate = roadrunner.getPoseEstimate();
        DMatrixRMaj x = new DMatrixRMaj(new double[][]{{firstEstimate.getX()},
                {firstEstimate.getY()},
                {firstEstimate.getHeading()}});

        DMatrixRMaj p = new DMatrixRMaj(new double[][]{{1, 0, 0},
                {0, 1, 0},
                {0, 0, 1}});
        roadrunner.setPoseEstimate(startPosition.toPose2d());
        estimator.setState(x, p);

        DMatrixRMaj F = new DMatrixRMaj(new double[][]{{1, 0, 0},
                {0, 1, 0},
                {0, 0, 1}});
        DMatrixRMaj Q = new DMatrixRMaj(new double[][]{{0.1, 0, 0},
                {0, 0.1, 0},
                {0, 0, 0.1}});

        DMatrixRMaj H = new DMatrixRMaj(new double[][]{{0, 0, 0}, {0, 0, 0}, {0, 0, 1}});
        estimator.configure(F, Q, H);


    }

    public void update() {


        roadrunner.updatePoseEstimate();

        Pose2d firstEstimate = roadrunner.getPoseEstimate();
        DMatrixRMaj x = new DMatrixRMaj(new double[][]{{firstEstimate.getX()},
                {firstEstimate.getY()},
                {firstEstimate.getHeading()}});

        DMatrixRMaj z = new DMatrixRMaj(new double[][]{{0, 0, imu.getAngularOrientation().firstAngle}});

        DMatrixRMaj r = new DMatrixRMaj(new double[][]{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}});

        estimator.setState(x, estimator.getCovariance());
        estimator.predict();
        estimator.update(z, r);
        DMatrixRMaj state = estimator.getState();

        loop += 1;

        if (loop % 10 == 0) {

            roadrunner.setPoseEstimate(new Pose2d(firstEstimate.getX(),
                    firstEstimate.getY(), utils.normalizeAngleRR(state.get(2, 0))));


            loop = 0;

        }

    }

    public Vector3D getPoseEstimate() {
        return new Vector3D(roadrunner.getPoseEstimate());
    }

    public Pose2d getPoseVelocity() {
        return roadrunner.getPoseVelocity();
    }

    public void setPoseEstimate(Vector3D estimate) {
        roadrunner.setPoseEstimate(estimate.toPose2d());
    }

}
