package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Geometry.Vector3D;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;

import java.util.ArrayList;


import android.os.Build;

import androidx.annotation.RequiresApi;

import homeostasis.Filters.SISOKalmanFilter;

import static org.firstinspires.ftc.teamcode.Utils.utils.AngleWrap;
import static org.firstinspires.ftc.teamcode.Utils.utils.drawRobotBlue;
import static org.firstinspires.ftc.teamcode.Utils.utils.plotVector;
@Config
public class DistanceSensorLocalization implements subsystem{

	public Rev2mDistanceSensor leftSensor;
	public Rev2mDistanceSensor rightSensor;
	public Rev2mDistanceSensor rearSensor;
	public final double TILE_SIZE = 24;
	public final double maximumAngle = Math.toRadians(15);

	final double leftDistanceFromCenter = 5 + (1/8.0);
	final double leftDistanceFromEdge = 12.0;
	public static double frontDistanceFromCenter = 4.5;
	final double frontDistanceFromEdge = -0.5;


	public Vector3D leftDistanceSensorRobotRelative = new Vector3D(0,0,Math.toRadians(0.0));
	public Vector3D rearDistanceSensorRobotRelative = new Vector3D(0,0,Math.toRadians(0.0));
	public Vector3D rightDistanceSensorRobotRelative = new Vector3D(0,0,Math.toRadians(0.0));

	public double leftDistance = 0;
	public double rightDistance = 0;
	public double rearDistance = 0;

	ThreeWheelOdometry odom;

	ArrayList<Vector3D> previousVectors = new ArrayList<>();
	double Q = 7;
	double R = 30;
	int N = 3;

	double cutoffDistanceMAX = 70;
	double minDistance = 10;

	SISOKalmanFilter estimatorX = new SISOKalmanFilter(Q,R);
	SISOKalmanFilter estimatorY = new SISOKalmanFilter(Q,R);

	public DistanceSensorLocalization(ThreeWheelOdometry odom) {
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


		readSensors();

		if (leftDistance >= cutoffDistanceMAX || minDistance >= leftDistance) return;
		if (rearDistance >= cutoffDistanceMAX || minDistance >= rearDistance) return;



		Vector3D robotPose = odom.subsystemState();
		System.out.println("abs angle: " + Math.abs(AngleWrap(robotPose.getAngleRadians())) + " cutoff is "  + maximumAngle);
		if (Math.abs(AngleWrap(robotPose.getAngleRadians())) > maximumAngle) return;



		double x = rearDistance * Math.cos(robotPose.getAngleRadians() + rearDistanceSensorRobotRelative.getAngleRadians());
		double y = leftDistance * Math.cos(robotPose.getAngleRadians() + leftDistanceSensorRobotRelative.getAngleRadians());


		double x_field = (TILE_SIZE * 3) - (x - frontDistanceFromEdge + frontDistanceFromCenter);
		double y_field = -(TILE_SIZE * 3) + (y - leftDistanceFromEdge + leftDistanceFromCenter);


		Vector3D estimatedPose = new Vector3D(x_field, y_field, robotPose.getAngleRadians());
		plotVector(estimatedPose,"distance sensor pose estimate", Dashboard.packet);

		drawRobotBlue(estimatedPose, Dashboard.packet);

		double xPoseEstimate = estimatorX.updateKalmanMeasurements(robotPose.getX(), estimatedPose.getX());
		double yPoseEstimate = estimatorY.updateKalmanMeasurements(robotPose.getY(), estimatedPose.getY());

		odom.setYPose(yPoseEstimate);
		odom.setXPose(xPoseEstimate);

	}
}
