package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.filter.KalmanFilter;
import org.firstinspires.ftc.teamcode.geometry.Vector3D;

import java.util.ArrayList;
import java.util.Vector;

import static org.firstinspires.ftc.teamcode.utils.utils.connectVectors;
import static org.firstinspires.ftc.teamcode.utils.utils.drawRobotBlue;
import static org.firstinspires.ftc.teamcode.utils.utils.plotVector;
import static org.firstinspires.ftc.teamcode.utils.utils.visualizeVector;

import android.os.Build;

import androidx.annotation.RequiresApi;

import homeostasis.Filters.LeastSquaresKalmanFilter;
import homeostasis.Filters.SISOKalmanFilter;

public class DistanceSensorLocalization implements subsystem{

	public Rev2mDistanceSensor leftSensor;
	public Rev2mDistanceSensor rightSensor;
	public Rev2mDistanceSensor rearSensor;
	public final double TILE_SIZE = 24;

	public Vector3D leftDistanceSensorRobotRelative = new Vector3D(0,0,Math.toRadians(-34));
	public Vector3D rearDistanceSensorRobotRelative = new Vector3D(0,0,Math.toRadians(0));
	public Vector3D rightDistanceSensorRobotRelative = new Vector3D(0,0,Math.toRadians(120));

	public double leftDistance = 0;
	public double rightDistance = 0;
	public double rearDistance = 0;

	DifferentialDriveOdometry odom;

	ArrayList<Vector3D> previousVectors = new ArrayList<>();
	double Q = 0.5;
	double R = 30;
	int N = 3;

	double cutoffDistance = 322;
	double minDistance = 10;

	SISOKalmanFilter estimatorX = new SISOKalmanFilter(Q,R);
	SISOKalmanFilter estimatorY = new SISOKalmanFilter(Q,R);

	public DistanceSensorLocalization(DifferentialDriveOdometry odom) {
		this.odom = odom;
	}


	double hz = 10;
	double delay = 1000 / hz;

	ElapsedTime timer = new ElapsedTime();

	@Override
	public void init(HardwareMap hwmap) {

		this.leftSensor = hwmap.get(Rev2mDistanceSensor.class, "LeftDistance");
		this.rightSensor = hwmap.get(Rev2mDistanceSensor.class, "RightDistance");
		this.rearSensor = hwmap.get(Rev2mDistanceSensor.class, "RearDistance");
		timer.reset();

	}

	@Override
	public void initNoReset(HardwareMap hwmap) {
		init(hwmap);
	}

	@RequiresApi(api = Build.VERSION_CODES.N)
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

	@RequiresApi(api = Build.VERSION_CODES.N)
	public void calculatePositions() {
		System.out.println("current time for scheduling is " + timer.milliseconds());
		if (timer.milliseconds() < delay) return;
		timer.reset();

		Vector3D robotPose = odom.subsystemState();


		if (leftDistance >= cutoffDistance || minDistance >= leftDistance) return;
		if (rearDistance >= cutoffDistance || minDistance >= rearDistance) return;


		double x = rearDistance * Math.cos(robotPose.getAngleRadians() + rearDistanceSensorRobotRelative.getAngleRadians());
		double y = leftDistance * Math.cos(robotPose.getAngleRadians() + leftDistanceSensorRobotRelative.getAngleRadians());


		double x_field = (TILE_SIZE * 3) - x;
		double y_field = -(TILE_SIZE * 3) + y;

		Vector3D distancesFromWall = new Vector3D(x,y,0);
		plotVector(distancesFromWall,"distances from wall", Dashboard.packet);

		Vector3D estimatedPose = new Vector3D(x_field, y_field, robotPose.getAngleRadians());
		plotVector(estimatedPose,"distance sensor pose estimate", Dashboard.packet);

		drawRobotBlue(estimatedPose, Dashboard.packet);

		// kalman filtering

		double xPoseEstimate = estimatorX.updateKalmanMeasurements(robotPose.getX(), estimatedPose.getX());
		double yPoseEstimate = estimatorY.updateKalmanMeasurements(robotPose.getY(), estimatedPose.getY());


		odom.setXPose(xPoseEstimate);
		odom.setYPose(yPoseEstimate);




	}
}
