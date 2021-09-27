package org.firstinspires.ftc.teamcode.classicalControl;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.geometry.Vector3D;
import org.firstinspires.ftc.teamcode.subsystems.dashboard;
import org.firstinspires.ftc.teamcode.subsystems.robot;

import static org.firstinspires.ftc.teamcode.utils.utils.drawRobotGreen;
import static org.firstinspires.ftc.teamcode.utils.utils.normalizedHeadingError;

public class DifferentialDriveController {

	PIDFCoeffecients thetaCoefficients = new PIDFCoeffecients(0.8,0.01,0.155);
	PIDFCoeffecients omegaCoefficients = new PIDFCoeffecients(1,0,0.115);
	PIDFCoeffecients driveCoefficients = new PIDFCoeffecients(0.55,0,0.115);
	public static final double MAX_VELO = 75;
	ProfiledPIDController distanceController = new ProfiledPIDController(driveCoefficients, MAX_VELO, 55);
	NonlinearPID thetaController = new NonlinearPID(thetaCoefficients);
	NonlinearPID omegaController = new NonlinearPID(omegaCoefficients);

	protected double Kp3 = 0.23;
	protected double Kp5 = Kp3 * 1.5;
	protected double Kp6 = 9;
	protected double threshold = 2.5;
	protected robot robot;

	protected Vector3D output;

	public DifferentialDriveController(robot robot) {
		this.robot = robot;
		thetaController.setIntegralSumLimit(0.1);
		thetaController.setLimitIntegralSum(true);
		thetaController.setIntegralResetOnSetpointChange(true);
		thetaController.setIntegralZeroCrossoverDetection(true);
		thetaController.setEnableLowPassDerivative(false);
		omegaController.setEnableLowPassDerivative(false);
	}

	/**
	 * calculate robot relative velocities for the system
	 * @param positionSetpoint where the robot should be
	 * @return the motor command to move the robot in robot relative space
	 */
	public Vector3D controllerOutput(Vector3D positionSetpoint) {

		Vector3D positionError = positionSetpoint.getError(robot.getRobotPose());

		double distance = Math.sqrt(Math.pow(positionError.getX(),2) + Math.pow(positionError.getY(),2));
		double omega1 = Math.atan2(positionError.getY(),positionError.getX());
		double omega2 = Math.atan2(positionError.getY(),positionError.getX()) - Math.toRadians(180);
		double omegaError1 = normalizedHeadingError(omega1, robot.getRobotPose().getAngleRadians());
		double omegaError2 = normalizedHeadingError(omega2, robot.getRobotPose().getAngleRadians());
		double omegaError;

		double driveMultiplier = 1;
		if (Math.abs(omegaError1) < Math.abs(omegaError2) || Math.abs(omegaError1) - Math.abs(omegaError2) < Math.toRadians(20)) {
			omegaError = omegaError1;
		} else {
			omegaError = omegaError2;
			driveMultiplier = -1;

		}
		double driveComponent = driveMultiplier * distanceController.calculateProfiledOutput(0,-distance) / (Kp6 * Range.clip( Math.abs(omegaError),1,Math.PI * 2));
		double omegaComponent = Math.min(Kp3 * distance, 1) * omegaController.calculateOutput(-omegaError);
		double thetaComponent = thetaController.calculateOutput(-positionError.getAngleRadians()) / Math.max(distance * Kp5,1);
		if (distance < threshold) {
			driveComponent = 0;
			omegaComponent = 0;
		} else {
			thetaComponent = 0;
		}
		return new Vector3D(Range.clip(driveComponent,-1,1), 0,Range.clip(omegaComponent + thetaComponent,-1,1));


	}

	/**
	 * drive the robot to a desired position on the field using the control law defined in controllerOutput
	 * @param position the desired robot position
	 */
	public boolean driveToPosition(Vector3D position) {
		output = controllerOutput(position);
		robot.driveTrain.robotRelative(output.getX(),output.getAngleRadians());
		drawRobotGreen(position, dashboard.packet);
		return (robot.getRobotPose().distanceToPose(position) < threshold + 1.5
				&& Math.abs(robot.getRobotPose().getError(position).getAngleRadians())
				< Math.toRadians(4) && Math.abs(robot.getVelocity().getAngleRadians()) < 0.002);
	}

	public void driveToAngle(Vector3D position) {
		position.setX(robot.getRobotPose().getX());
		position.setY(robot.getRobotPose().getY());
		driveToPosition(position);
	}





}
