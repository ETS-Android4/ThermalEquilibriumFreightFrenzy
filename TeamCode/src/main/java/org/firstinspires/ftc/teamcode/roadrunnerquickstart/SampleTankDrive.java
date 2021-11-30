package org.firstinspires.ftc.teamcode.roadrunnerquickstart;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.TankDrive;
import com.acmerobotics.roadrunner.followers.TankPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.teamcode.filter.LowPassFilter;
import org.firstinspires.ftc.teamcode.geometry.Vector3D;
import org.firstinspires.ftc.teamcode.roadrunnerquickstart.util.LynxModuleUtil;

import java.util.Arrays;
import java.util.List;

import static org.firstinspires.ftc.teamcode.roadrunnerquickstart.DriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.roadrunnerquickstart.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.roadrunnerquickstart.DriveConstants.encoderTicksToInches;


/*
 * Simple tank drive hardware implementation for REV hardware.
 */
@Config
public class SampleTankDrive extends TankDrive {
	public static PIDCoefficients AXIAL_PID = new PIDCoefficients(0, 0, 0);
	public static PIDCoefficients CROSS_TRACK_PID = new PIDCoefficients(0, 0, 0);
	public static PIDCoefficients HEADING_PID = new PIDCoefficients(0, 0, 0);
	double lowPassGain = 0.98;
	private LowPassFilter xAccelFilter = new LowPassFilter(lowPassGain);
	private LowPassFilter yAccelFilter = new LowPassFilter(lowPassGain);
	public static double VX_WEIGHT = 1;
	public static double OMEGA_WEIGHT = 1;


	private TrajectoryFollower follower;

	private List<DcMotorEx> motors, leftMotors, rightMotors;
	private BNO055IMU imu;

	private VoltageSensor batteryVoltageSensor;

	public SampleTankDrive(HardwareMap hardwareMap) {
		super(DriveConstants.kV, DriveConstants.kA, DriveConstants.kStatic, TRACK_WIDTH);

		follower = new TankPIDVAFollower(AXIAL_PID, CROSS_TRACK_PID,
				new Pose2d(0.5, 0.5, Math.toRadians(5.0)), 0.5);

		LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

		batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

		for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
			module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
		}

		// TODO: adjust the names of the following hardware devices to match your configuration
		imu = hardwareMap.get(BNO055IMU.class, "imu");
		BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
		parameters.mode = BNO055IMU.SensorMode.IMU;
		parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
		imu.initialize(parameters);


		// TODO: if your hub is mounted vertically, remap the IMU axes so that the z-axis points
		// upward (normal to the floor) using a command like the following:
		// BNO055IMUUtil.remapAxes(imu, AxesOrder.XYZ, AxesSigns.NPN);

		// add/remove motors depending on your robot (e.g., 6WD)
		DcMotorEx leftFront = hardwareMap.get(DcMotorEx.class, "FrontLeft");
		DcMotorEx leftRear = hardwareMap.get(DcMotorEx.class, "BackLeft");
		DcMotorEx rightRear = hardwareMap.get(DcMotorEx.class, "BackRight");
		DcMotorEx rightFront = hardwareMap.get(DcMotorEx.class, "FrontRight");

		motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);
		leftMotors = Arrays.asList(leftFront, leftRear);
		rightMotors = Arrays.asList(rightFront, rightRear);

		for (DcMotorEx motor : motors) {
			MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
			motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
			motor.setMotorType(motorConfigurationType);
		}

		if (RUN_USING_ENCODER) {
			setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		}

		setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


		// TODO: reverse any motors using DcMotor.setDirection()

		// TODO: if desired, use setLocalizer() to change the localization method
		// for instance, setLocalizer(new ThreeTrackingWheelLocalizer(...));

	}




	public void update() {
		updatePoseEstimate();
	}


	public void setMode(DcMotor.RunMode runMode) {
		for (DcMotorEx motor : motors) {
			motor.setMode(runMode);
		}
	}

	public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
		for (DcMotorEx motor : motors) {
			motor.setZeroPowerBehavior(zeroPowerBehavior);
		}
	}

	public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
		PIDFCoefficients compensatedCoefficients = new PIDFCoefficients(
				coefficients.p, coefficients.i, coefficients.d,
				coefficients.f * 12 / batteryVoltageSensor.getVoltage()
		);
		for (DcMotorEx motor : motors) {
			motor.setPIDFCoefficients(runMode, compensatedCoefficients);
		}
	}

	public void setWeightedDrivePower(Pose2d drivePower) {
		Pose2d vel = drivePower;

		if (Math.abs(drivePower.getX()) + Math.abs(drivePower.getHeading()) > 1) {
			// re-normalize the powers according to the weights
			double denom = VX_WEIGHT * Math.abs(drivePower.getX())
					+ OMEGA_WEIGHT * Math.abs(drivePower.getHeading());

			vel = new Pose2d(
					VX_WEIGHT * drivePower.getX(),
					0,
					OMEGA_WEIGHT * drivePower.getHeading()
			).div(denom);
		}

		setDrivePower(vel);
	}

	@NonNull
	@Override
	public List<Double> getWheelPositions() {
		double leftSum = 0, rightSum = 0;
//		for (DcMotorEx leftMotor : leftMotors) {
//			leftSum += encoderTicksToInches(leftMotor.getCurrentPosition());
//		}
    	leftSum += encoderTicksToInches(leftMotors.get(0).getCurrentPosition());

//		for (DcMotorEx rightMotor : rightMotors) {
//			rightSum += encoderTicksToInches(rightMotor.getCurrentPosition());
//		}
		rightSum += encoderTicksToInches(rightMotors.get(0).getCurrentPosition());

		return Arrays.asList(leftSum, rightSum);
	}

	public List<Double> getWheelVelocities() {
		double leftSum = 0, rightSum = 0;
		for (DcMotorEx leftMotor : leftMotors) {
			leftSum += encoderTicksToInches(leftMotor.getVelocity());
		}
		for (DcMotorEx rightMotor : rightMotors) {
			rightSum += encoderTicksToInches(rightMotor.getVelocity());
		}
		return Arrays.asList(leftSum / leftMotors.size(), rightSum / rightMotors.size());
	}

	@Override
	public void setMotorPowers(double v, double v1) {
		for (DcMotorEx leftMotor : leftMotors) {
			leftMotor.setPower(v);
		}
		for (DcMotorEx rightMotor : rightMotors) {
			rightMotor.setPower(v1);
		}
	}

	@Override
	public double getRawExternalHeading() {
		return imu.getAngularOrientation().firstAngle;
	}

	@Override
	public Double getExternalHeadingVelocity() {
		// TODO: This must be changed to match your configuration
		//                           | Z axis
		//                           |
		//     (Motor Port Side)     |   / X axis
		//                       ____|__/____
		//          Y axis     / *   | /    /|   (IO Side)
		//          _________ /______|/    //      I2C
		//                   /___________ //     Digital
		//                  |____________|/      Analog
		//
		//                 (Servo Port Side)
		//
		// The positive x axis points toward the USB port(s)
		//
		// Adjust the axis rotation rate as necessary
		// Rotate about the z axis is the default assuming your REV Hub/Control Hub is laying
		// flat on a surface

		return (double) imu.getAngularVelocity().zRotationRate;
	}

	public Vector3D getIMUAccel() {
		Acceleration accel = imu.getLinearAcceleration();

		return new Vector3D(accel.xAccel,accel.yAccel,accel.zAccel,0,0).scale(39.3701 * 9.83,false);
	}


}

