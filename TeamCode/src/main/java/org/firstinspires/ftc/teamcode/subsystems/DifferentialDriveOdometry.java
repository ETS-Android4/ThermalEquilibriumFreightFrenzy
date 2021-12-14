package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.NaiveAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.geometry.Vector3D;
import org.firstinspires.ftc.teamcode.roadrunnerquickstart.util.AxesSigns;
import org.firstinspires.ftc.teamcode.roadrunnerquickstart.util.BNO055IMUUtil;

import homeostasis.Filters.AngleKalmanFilter;

import static org.firstinspires.ftc.teamcode.subsystems.Robot.isCompBot;
import static org.firstinspires.ftc.teamcode.utils.utils.AngleWrap;
import static org.firstinspires.ftc.teamcode.utils.utils.drawRobot;
import static org.firstinspires.ftc.teamcode.utils.utils.normalizeAngleRR;

public class DifferentialDriveOdometry implements subsystem {


	public DcMotorEx FrontLeft;
	public DcMotorEx FrontRight;
	protected Vector3D positionEstimate = new Vector3D();
	protected Vector3D positionEstimateDeltaFieldRelative = new Vector3D();
	protected Vector3D positionEstimateDeltaRobotRelative = new Vector3D();
	private double pitchAngle = 0;
	private double leftPrev = 0;
	private double rightPrev = 0;
	private final double gearRatio;
	double trackWidth;
	double testBotTrackWidth = 15.543307;
	double compBotTrackWidth = 16;
	private BNO055IMU imu;
	protected Vector3D initialPosition = new Vector3D();
	protected double IMU_angle = 0;
	double encoderAngle = 0;
	double xDot = 0;
	protected long counter = 0;
	protected AngleKalmanFilter kalmanFilter;


	double angularVelocity = 0;

	/**
	 * initialize a differential drive robot with odometry
	 */
	public DifferentialDriveOdometry() {
		if (isCompBot) {
			trackWidth = compBotTrackWidth;
			gearRatio = 20.0 / 24.0;
		} else {
			trackWidth = testBotTrackWidth;
			gearRatio = 1;
		}
		kalmanFilter = new AngleKalmanFilter(0);

	}

	@Override
	public void init(HardwareMap hwmap) {

		if (isCompBot) {
			imu = hwmap.get(BNO055IMU.class, "imu 1");
		} else {
			imu = hwmap.get(BNO055IMU.class, "imu");
		}
		BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
		parameters.mode = BNO055IMU.SensorMode.NDOF;
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

		updateIMU();
		double left = encoderTicksToInches(FrontLeft.getCurrentPosition());
		double right = encoderTicksToInches(FrontRight.getCurrentPosition());

		double leftVelo = encoderTicksToInches(FrontLeft.getVelocity());
		double rightVelo = encoderTicksToInches(FrontRight.getVelocity());

		double leftDelta = left - leftPrev;
		double rightDelta = right - rightPrev;
		leftPrev = left;
		rightPrev = right;

		double xDelta = (leftDelta + rightDelta) / 2;
		double yDelta = 0;
		double thetaDelta = (rightDelta - leftDelta) / (trackWidth);

		xDot = (leftVelo + rightVelo) / 2;

		encoderAngle += thetaDelta;
		encoderAngle = normalizeAngleRR(encoderAngle);

		positionEstimateDeltaRobotRelative = new Vector3D(xDelta, yDelta, thetaDelta);
		positionEstimate.setAngleRad(positionEstimate.getAngleRadians() + thetaDelta);

		positionEstimateDeltaFieldRelative = positionEstimateDeltaRobotRelative.rotateBy(positionEstimate.getAngleDegrees());
		positionEstimate = positionEstimate.add(positionEstimateDeltaFieldRelative);//positionEstimate.poseExponential(positionEstimateDeltaRobotRelative);

		double estimate = kalmanFilter.updateKalmanEstimate(encoderAngle, IMU_angle);

		positionEstimate.setAngleRad(estimate);

		drawRobot(positionEstimate, Dashboard.packet);
		Dashboard.packet.put("imu angle ", AngleWrap(IMU_angle));
		Dashboard.packet.put("estimated angle",estimate);
		Dashboard.packet.put("drive wheel angle", AngleWrap(encoderAngle));

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
		kalmanFilter.setX(positionEstimate.getAngleRadians());
	}

	/**
	 *
	 * @return the field relative pose delta
	 */
	public Vector3D getPositionEstimateDelta() {
		return positionEstimateDeltaFieldRelative;
	}

	public void updateIMU() {
		Orientation angle = imu.getAngularOrientation();
		IMU_angle = normalizeAngleRR(angle.firstAngle + initialPosition.getAngleRadians());//normalizeAngleRR(navx.subsystemState().getAngleRadians());
		Dashboard.packet.put("pitch angle", pitchAngle);
		angularVelocity = imu.getAngularVelocity().zRotationRate;
	}

	public Vector3D getVelocity() {
		return new Vector3D(xDot, 0, angularVelocity);
	}

	public double encoderTicksToInches(double ticks) {
		double WHEEL_RADIUS = 3.77953 / 2;
		double ticksPerRevolution = 28.0 * 13.7;
		return WHEEL_RADIUS * 2 * Math.PI * gearRatio * ticks / ticksPerRevolution;
	}


}
