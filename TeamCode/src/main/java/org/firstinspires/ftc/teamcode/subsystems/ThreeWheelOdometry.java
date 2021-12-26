package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Geometry.Vector3D;

import homeostasis.Filters.AngleKalmanFilter;

import static org.firstinspires.ftc.teamcode.subsystems.Robot.isCompBot;
import static org.firstinspires.ftc.teamcode.Utils.utils.AngleWrap;
import static org.firstinspires.ftc.teamcode.Utils.utils.drawRobot;
import static org.firstinspires.ftc.teamcode.Utils.utils.normalizeAngleRR;

public class ThreeWheelOdometry implements subsystem {


	public DcMotorEx LeftEncoder;
	public DcMotorEx RightEncoder;
	public DcMotorEx MiddleEncoder;
	protected Vector3D positionEstimate = new Vector3D();
	protected Vector3D positionEstimateDeltaFieldRelative = new Vector3D();
	protected Vector3D positionEstimateDeltaRobotRelative = new Vector3D();
	private double pitchAngle = 0;
	private double leftPrev = 0;
	private double rightPrev = 0;
	private double middlePrev = 0;
	private final double gearRatio;
	double trackWidth;
	double testBotTrackWidth = 15.543307;
	double compBotTrackWidth = 16; // TODO: fix this for real robot
	double middleWheelOffset = 5;  // TODO: fix this for real robot
	private BNO055IMU imu;
	protected Vector3D initialPosition = new Vector3D();
	protected double IMU_angle = 0;
	double encoderAngle = 0;
	double xDot = 0;
	protected long counter = 0;
	protected AngleKalmanFilter kalmanFilter;


	protected OdomState state = OdomState.DEPLOYED;

	double angularVelocity = 0;

	/**
	 * initialize a differential drive robot with odometry
	 */
	public ThreeWheelOdometry() {
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
			imu = hwmap.get(BNO055IMU.class, "imu");
		} else {
			imu = hwmap.get(BNO055IMU.class, "imu");
		}
		BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
		parameters.mode = BNO055IMU.SensorMode.NDOF;
		parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
		imu.initialize(parameters);
		LeftEncoder = hwmap.get(DcMotorEx.class, "FrontLeft");
		RightEncoder = hwmap.get(DcMotorEx.class, "FrontRight");
		MiddleEncoder = hwmap.get(DcMotorEx.class, "BackLeft");
	}

	@Override
	public void initNoReset(HardwareMap hwmap) {
		init(hwmap);
	}

	@Override
	public void update() {
		switch (state) {
			case DEPLOYED:
				updateIMU();
				break;
			case RETRACTED:
				deployedUpdate();
				break;
		}
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
		return new Vector3D(xDot, 0, angularVelocity).rotateBy(positionEstimate.getAngleDegrees());
	}


	public double encoderTicksToInches(double ticks) {
		double WHEEL_RADIUS = 1.49606 / 2; // 38 mm wheel
		double ticksPerRevolution = 8192;
		return WHEEL_RADIUS * 2 * Math.PI * gearRatio * ticks / ticksPerRevolution;
	}


	public enum OdomState {
		DEPLOYED,
		RETRACTED
	}

	public void setRetractionState(OdomState state) {
		this.state = state;
	}

	public void deployedUpdate() {
		double left = encoderTicksToInches(LeftEncoder.getCurrentPosition());
		double right = encoderTicksToInches(RightEncoder.getCurrentPosition());
		double middle = encoderTicksToInches(MiddleEncoder.getCurrentPosition());

		double leftVelo = encoderTicksToInches(LeftEncoder.getVelocity()); // TODO This will break with the rev encoder
		double rightVelo = encoderTicksToInches(RightEncoder.getVelocity());

		double leftDelta = left - leftPrev;
		double rightDelta = right - rightPrev;
		double middleDelta = middle - middlePrev;

		leftPrev = left;
		rightPrev = right;
		middlePrev = middle;

		double xDelta = (leftDelta + rightDelta) / 2;
		double yDelta = (middleWheelOffset / trackWidth) * (leftDelta - rightDelta) + middleDelta;
		double thetaDelta = (rightDelta - leftDelta) / (trackWidth);

		xDot = (leftVelo + rightVelo) / 2;

		encoderAngle += thetaDelta;
		encoderAngle = normalizeAngleRR(encoderAngle);

		positionEstimateDeltaRobotRelative = new Vector3D(xDelta, yDelta, thetaDelta);
		positionEstimate.setAngleRad(positionEstimate.getAngleRadians() + thetaDelta);

		positionEstimateDeltaFieldRelative = positionEstimateDeltaRobotRelative.rotateBy(positionEstimate.getAngleDegrees());
		positionEstimate = positionEstimate.add(positionEstimateDeltaFieldRelative);//positionEstimate.poseExponential(positionEstimateDeltaRobotRelative);

		drawRobot(positionEstimate, Dashboard.packet);

	}
}
