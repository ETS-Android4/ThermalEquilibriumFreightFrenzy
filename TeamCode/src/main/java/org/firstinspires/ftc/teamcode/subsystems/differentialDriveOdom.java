package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.teamcode.geometry.Vector3D;
import org.firstinspires.ftc.teamcode.utils.SizedStack;

import static org.firstinspires.ftc.teamcode.roadrunnerquickstart.DriveConstants.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.utils.utils.drawRobot;
import static org.firstinspires.ftc.teamcode.utils.utils.normalizeAngle;
import static org.firstinspires.ftc.teamcode.utils.utils.normalizeAngleRR;

public class differentialDriveOdom implements subsystem {

	public DcMotorEx FrontLeft;
	public DcMotorEx FrontRight;
	protected Vector3D positionEstimate = new Vector3D();
	protected Vector3D positionEstimateDeltaFieldRelative = new Vector3D();
	protected Vector3D positionEstimateDeltaRobotRelative = new Vector3D();
	private double leftPrev = 0;
	private double rightPrev = 0;
	double trackWidth = 18;
	private BNO055IMU imu;
	protected Vector3D initialPosition = new Vector3D();
	protected Vector3D accels = new Vector3D();
	protected double IMU_angle = 0;
	SizedStack<Vector3D> accels = new SizedStack<>(3);
	SizedStack<Double> dts = new SizedStack<>(3);

	private boolean hasRun = false;

	double accelDt = 0;
	double imuYDx = 0; // change in y position as estimated by the IMU
	ElapsedTime timer = new ElapsedTime();


	/**
	 * initialize a differential drive robot with odometry
	 */
	public differentialDriveOdom() {

	}

	@Override
	public void init(HardwareMap hwmap) {

		imu = hwmap.get(BNO055IMU.class, "imu");
		BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
		parameters.mode = BNO055IMU.SensorMode.IMU;
		parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
		imu.initialize(parameters);
		FrontLeft = hwmap.get(DcMotorEx.class, "FrontLeft");
		FrontRight = hwmap.get(DcMotorEx.class, "FrontRight");


	}

	@Override
	public void initNoReset(HardwareMap hwmap) {
		init(hwmap);
	}

	@Override
	public void update() {


		if (!hasRun)
		{
			timer.reset();
			hasRun = true;
		}
		double left = encoderTicksToInches(FrontLeft.getCurrentPosition());
		double right = encoderTicksToInches(FrontRight.getCurrentPosition());
		double leftDelta = left - leftPrev;
		double rightDelta = right - rightPrev;
		leftPrev = left;
		rightPrev = right;
		updateIMU();

		double xDelta = (leftDelta + rightDelta) / 2;
		double yDelta = imuYDx;
		double thetaDelta = (rightDelta - leftDelta) / (trackWidth);

		positionEstimateDeltaRobotRelative = new Vector3D(xDelta,yDelta,thetaDelta);

		// we need some second order dynamics imo (in my option)
		//positionEstimateDeltaFieldRelative = positionEstimateDeltaRobotRelative.rotateBy(positionEstimate.getAngleDegrees());

		positionEstimate = positionEstimate.poseExponential(positionEstimateDeltaRobotRelative);
		positionEstimate.setAngleRad(IMU_angle);
		drawRobot(positionEstimate,dashboard.packet);
		System.out.println("the estimated pose from the custom odom is " + positionEstimate);


	}



	/**
	 * position estimate of the robot
	 * @return get the robots position estimate
	 */
	@Override
	public Vector3D subsystemState() {
		return positionEstimate;
	}

	/**
	 * set the pose estimate based on a vector3d object
	 * @param positionEstimate pose estimate
	 */
	public void setPositionEstimate(Vector3D positionEstimate) {
		this.initialPosition = positionEstimate;
		this.positionEstimate = positionEstimate;
	}

	/**
	 *
	 * @return
	 */
	public Vector3D getPositionEstimateDelta() {
		return positionEstimateDeltaFieldRelative;
	}

	/**
	 * position estimate robot relative
	 * @return the robot relative position estimate
	 */
	public Vector3D robotRelativeDelta() {
		return positionEstimateDeltaRobotRelative;
	}


	public void updateIMU() {

		IMU_angle = normalizeAngleRR(imu.getAngularOrientation().firstAngle + initialPosition.getAngleRadians());
		Acceleration accel = imu.getLinearAcceleration();
		accelDt = timer.seconds();
		timer.reset();

		accels = new Vector3D(0,accel.yAccel * 386.08858267717,0);


		imuYDx = accels.getY() * Math.pow(accelDt,2);
		System.out.println("accel dt is " + accelDt + " IMUYdt is " + imuYDx);

	}





}
