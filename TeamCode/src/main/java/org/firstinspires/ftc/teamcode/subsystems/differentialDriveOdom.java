package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.geometry.Vector3D;

import homeostasis.Filters.angleKalmanFilter;

import static org.firstinspires.ftc.teamcode.roadrunnerquickstart.DriveConstants.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.utils.utils.drawRobot;
import static org.firstinspires.ftc.teamcode.utils.utils.drawRobotBlue;
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
	protected double IMU_angle = 0;
	protected double revIMUAngle = 0;
	protected double kalmanFilterAngleEstimate = 0;
	protected angleKalmanFilter angleKf;

	protected navxIMU navx;

	double angularVelocity = 0;

	/**
	 * initialize a differential drive robot with odometry
	 */
	public differentialDriveOdom(navxIMU navx) {
		this.navx = navx;
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
		angleKf = new angleKalmanFilter(0);

	}

	@Override
	public void initNoReset(HardwareMap hwmap) {
		init(hwmap);
	}

	@Override
	public void update() {


		double left = encoderTicksToInches(FrontLeft.getCurrentPosition());
		double right = encoderTicksToInches(FrontRight.getCurrentPosition());
		double leftDelta = left - leftPrev;
		double rightDelta = right - rightPrev;
		leftPrev = left;
		rightPrev = right;
		updateIMU();

		double xDelta = (leftDelta + rightDelta) / 2;
		double yDelta = 0;
		double thetaDelta = (rightDelta - leftDelta) / (trackWidth);

		positionEstimateDeltaRobotRelative = new Vector3D(xDelta,yDelta,thetaDelta);

		// we need some second order dynamics imo (in my option)
		positionEstimateDeltaFieldRelative = positionEstimateDeltaRobotRelative.rotateBy(positionEstimate.getAngleDegrees());
		positionEstimate = positionEstimate.add(positionEstimateDeltaFieldRelative);//positionEstimate.poseExponential(positionEstimateDeltaRobotRelative);

		kalmanFilterAngleEstimate = angleKf.updateKalmanEstimate(thetaDelta,IMU_angle);
		positionEstimate.setAngleRad(kalmanFilterAngleEstimate);
		drawRobot(positionEstimate,dashboard.packet);



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
	 * @return the field relative pose delta
	 */
	public Vector3D getPositionEstimateDelta() {
		return positionEstimateDeltaFieldRelative;
	}

	public void updateIMU() {
		IMU_angle = navx.subsystemState().getAngleRadians();
		revIMUAngle = normalizeAngleRR(imu.getAngularOrientation().firstAngle + initialPosition.getAngleRadians());
		//IMU_angle = normalizeAngleRR(imu.getAngularOrientation().firstAngle + initialPosition.getAngleRadians());
	}

	public Vector3D getVelocity() {
		return new Vector3D(0,0,angularVelocity);
	}




}
