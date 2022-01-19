package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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

public class DistanceSensorLocalization implements subsystem{

	public Rev2mDistanceSensor leftSensor;
	public Rev2mDistanceSensor rightSensor;
	public Rev2mDistanceSensor rearSensor;
	public final double TILE_SIZE = 24;

	public Vector3D leftDistanceSensorRobotRelative = new Vector3D(0,0,Math.toRadians(-120));
	public Vector3D rearDistanceSensorRobotRelative = new Vector3D(0,0,Math.toRadians(0));
	public Vector3D rightDistanceSensorRobotRelative = new Vector3D(0,0,Math.toRadians(120));

	public double leftDistance = 0;
	public double rightDistance = 0;
	public double rearDistance = 0;

	DifferentialDriveOdometry odom;

	ArrayList<Vector3D> previousVectors = new ArrayList<>();
	double Q = 0.5;
	double R = 3;
	int N = 3;

	LeastSquaresKalmanFilter xKf = new LeastSquaresKalmanFilter(Q,R,N, false);
	LeastSquaresKalmanFilter yKf = new LeastSquaresKalmanFilter(Q,R,N,false);

	public DistanceSensorLocalization(DifferentialDriveOdometry odom) {
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

		Vector3D robotPose = odom.subsystemState();

		double x = rearDistance * Math.cos(robotPose.getAngleRadians() + rearDistanceSensorRobotRelative.getAngleRadians());
		double y = leftDistance * Math.sin(robotPose.getAngleRadians() + leftDistanceSensorRobotRelative.getAngleRadians());
		x = xKf.update(x);
		y = xKf.update(y);

		double x_field = (TILE_SIZE * 3) - x;
		double y_field = -(TILE_SIZE * 3) - y;

		Vector3D distancesFromWall = new Vector3D(x,y,0);
		plotVector(distancesFromWall,"distances from wall", Dashboard.packet);

		Vector3D estimatedPose = new Vector3D(x_field, y_field, robotPose.getAngleRadians());
		plotVector(estimatedPose,"distance sensor pose estimate", Dashboard.packet);

		drawRobotBlue(estimatedPose, Dashboard.packet);




	}
}
