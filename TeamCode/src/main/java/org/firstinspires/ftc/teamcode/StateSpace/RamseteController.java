package org.firstinspires.ftc.teamcode.StateSpace;

import org.firstinspires.ftc.teamcode.geometry.Vector3D;
import org.firstinspires.ftc.teamcode.subsystems.dashboard;
import org.firstinspires.ftc.teamcode.subsystems.robot;

import static org.firstinspires.ftc.teamcode.utils.utils.drawRobotGreen;

public class RamseteController {


	protected Vector3D output;

	protected robot robot;
	private double threshold = 3;
	protected double B = 2 / 39.37;
	protected double Zeta = 0.7 / 39.37;

	public RamseteController(robot robot) {
		this.robot = robot;
	}

	/**
	 * calculate robot relative velocities for the system
	 * @param positionSetpoint where the robot should be
	 * @return the motor command to move the robot in robot relative space
	 */
	public Vector3D controllerOutput(Vector3D positionSetpoint, Vector3D velocitySetpoint) {

		Vector3D posError = positionSetpoint.getError(robot.getRobotPose());

		double sinc;
		if (Math.abs(posError.getAngleRadians()) < 0.01) {
			sinc = 1.0 - posError.getAngleRadians() * posError.getAngleRadians() / 6.0;
		} else {
			sinc = Math.sin(posError.getAngleRadians()) / posError.getAngleRadians();
		}
		double veloSetSqr = Math.pow(velocitySetpoint.getAngleRadians(),2);
		double bVeloSetSqr = Math.pow(B * velocitySetpoint.getX(), 2);
		double k = 2 * Zeta * Math.sqrt(veloSetSqr + bVeloSetSqr);
		double w = velocitySetpoint.getAngleRadians() + k * posError.getAngleRadians() + B * velocitySetpoint.getX() * sinc * posError.getY();
		double v = velocitySetpoint.getX() * Math.cos(posError.getAngleRadians()) + k * posError.getX();

		return new Vector3D(v,0,w);


	}

	/**
	 * drive the robot to a desired position on the field using the control law defined in controllerOutput
	 * @param position the desired robot position
	 */
	public boolean driveToPosition(Vector3D position, Vector3D velocity) {
		output = controllerOutput(position, velocity).multiply(new Vector3D(0.005, 0.005, 0.005));
		robot.driveTrain.robotRelative(output.getX(),output.getAngleRadians());
		drawRobotGreen(position, dashboard.packet);
		return robot.getRobotPose().distanceToPose(position) < threshold + 1
				&& Math.abs(robot.getRobotPose().getError(position).getAngleRadians())
				< Math.toRadians(3);
	}


}
