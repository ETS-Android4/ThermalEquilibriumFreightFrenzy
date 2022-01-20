package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.geometry.Vector3D;

import homeostasis.Filters.AngleKalmanFilter;
import homeostasis.Filters.LeastSquaresKalmanFilter;

import static org.firstinspires.ftc.teamcode.subsystems.Robot.isCompBot;
import static org.firstinspires.ftc.teamcode.utils.utils.AngleWrap;
import static org.firstinspires.ftc.teamcode.utils.utils.drawRobot;
import static org.firstinspires.ftc.teamcode.utils.utils.normalizeAngleRR;

import android.os.Build;

import androidx.annotation.RequiresApi;

public class DifferentialDriveOdometry implements subsystem {


	private static double gearRatio = 1;
	public DcMotorEx FrontLeft;
	public DcMotorEx FrontRight;
	protected Vector3D positionEstimate = new Vector3D();
	protected Vector3D positionEstimateDeltaFieldRelative = new Vector3D();
	protected Vector3D positionEstimateDeltaRobotRelative = new Vector3D();
	private double pitchAngle = 0;
	private double pitchVelo = 0;
	private double leftPrev = 0;
	private double rightPrev = 0;
	double trackWidth;
	public static double testBotTrackWidth = 35.70453809697589;
	double compBotTrackWidth = 16;
	private BNO055IMU imu;
	protected Vector3D initialPosition = new Vector3D();
	protected double IMU_angle = 0;
	double encoderAngle = 0;
	double xDot = 0;

	protected AngleKalmanFilter kalmanFilter;
	protected LeastSquaresKalmanFilter forwardVelocityFilter;
	protected LeastSquaresKalmanFilter leftEncoderFilter;
	protected LeastSquaresKalmanFilter rightEncoderFilter;
	protected LeastSquaresKalmanFilter pitchVelocityFilter;

	protected double previousDeltaX;
	ElapsedTime timer = new ElapsedTime();

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
		forwardVelocityFilter = new LeastSquaresKalmanFilter(3.9,20,3,false);
		leftEncoderFilter = new LeastSquaresKalmanFilter(0.9,0.3,3,false);
		rightEncoderFilter = new LeastSquaresKalmanFilter(0.9,0.3,3,false);
		pitchVelocityFilter = new LeastSquaresKalmanFilter(1.9,10,2,false);
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

	@RequiresApi(api = Build.VERSION_CODES.N)
	@Override
	public void update() {

		updateIMU();
		double left = encoderTicksToInches(FrontLeft.getCurrentPosition());
		double right = encoderTicksToInches(FrontRight.getCurrentPosition());
		double leftVelo = encoderTicksToInches(FrontLeft.getVelocity());
		double rightVelo = encoderTicksToInches(FrontRight.getVelocity());
		double inbuiltVelo = (leftVelo + rightVelo) / 2;

		double leftDelta = left - leftPrev;
		double rightDelta = right - rightPrev;
		leftPrev = left;
		rightPrev = right;

		double xDelta = (leftDelta + rightDelta) / 2;
		double yDelta = 0;
		double thetaDelta = (rightDelta - leftDelta) / (trackWidth);

		xDot = (xDelta - previousDeltaX) / timer.seconds();

		timer.reset();
		double xDotFiltered = forwardVelocityFilter.update(inbuiltVelo);

		encoderAngle += thetaDelta;
		encoderAngle = normalizeAngleRR(encoderAngle);

		positionEstimateDeltaRobotRelative = new Vector3D(xDelta, yDelta, thetaDelta);
		positionEstimate.setAngleRad(positionEstimate.getAngleRadians() + thetaDelta);

		positionEstimateDeltaFieldRelative = positionEstimateDeltaRobotRelative.rotateBy(positionEstimate.getAngleDegrees());
		positionEstimate = positionEstimate.add(positionEstimateDeltaFieldRelative);//positionEstimate.poseExponential(positionEstimateDeltaRobotRelative);

		double estimate = IMU_angle;//kalmanFilter.updateKalmanEstimate(encoderAngle, IMU_angle);

		positionEstimate.setAngleRad(estimate);

		drawRobot(positionEstimate, Dashboard.packet);

//		Dashboard.packet.put("estimated angle",estimate);
		Dashboard.packet.put("imu angle ", AngleWrap(IMU_angle));
		Dashboard.packet.put("drive wheel angle", AngleWrap(encoderAngle));
		Dashboard.packet.put("measured x velocity", xDot);
		Dashboard.packet.put("estimated x velocity", xDotFiltered);
		Dashboard.packet.put("inbuilt x velocity", inbuiltVelo);

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

	@RequiresApi(api = Build.VERSION_CODES.N)
	public void updateIMU() {
		Orientation angle = imu.getAngularOrientation();
		pitchAngle = AngleWrap(angle.secondAngle);
		IMU_angle = normalizeAngleRR(angle.firstAngle + initialPosition.getAngleRadians());//normalizeAngleRR(navx.subsystemState().getAngleRadians());
		Dashboard.packet.put("pitch angle", pitchAngle);
		Dashboard.packet.put("pitch angle deg",Math.toDegrees(pitchAngle));
		System.out.println("pitch angle: " + pitchAngle + " pitch angle deg: " + Math.toDegrees(pitchAngle));
		angularVelocity = imu.getAngularVelocity().xRotationRate;
		AngularVelocity angularVelocity = imu.getAngularVelocity();
		if (!isCompBot) {
			pitchVelo = angularVelocity.zRotationRate;
		} else {
			pitchVelo = -angularVelocity.xRotationRate;
		}
		pitchVelo = pitchVelocityFilter.update(pitchVelo);
		Dashboard.packet.put("pitch velocity",pitchVelo);
		Dashboard.packet.put("angle x velo",angularVelocity.xRotationRate);
		Dashboard.packet.put("angle y velo",angularVelocity.yRotationRate);
		Dashboard.packet.put("angle z velo",angularVelocity.zRotationRate);
	}

	public Vector3D getVelocity() {
		return new Vector3D(xDot, 0, angularVelocity);
	}


	public static double encoderTicksToInches(double ticks) {
		double WHEEL_RADIUS = 3.77953 / 2;
		double ticksPerRevolution = 28.0 * 13.7;
		return WHEEL_RADIUS * 2 * Math.PI * gearRatio * ticks / ticksPerRevolution;
	}

	public double getPitchAngle() {
		return pitchAngle;
	}

	public double getPitchVelo() {
		return pitchVelo;
	}

	public void setXPose(double x) {
		this.positionEstimate.setX(x);
	}

	public void setYPose(double y) {
		this.positionEstimate.setY(y);
	}


}
