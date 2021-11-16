package org.firstinspires.ftc.teamcode.classicalControl;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.basedControl.basedControl;
import org.firstinspires.ftc.teamcode.geometry.Vector3D;
import org.firstinspires.ftc.teamcode.subsystems.dashboard;
import org.firstinspires.ftc.teamcode.subsystems.robot;

import static org.firstinspires.ftc.teamcode.utils.utils.drawRobotGreen;
import static org.firstinspires.ftc.teamcode.utils.utils.normalizedHeadingError;

public class DifferentialDriveController {


	PIDFCoefficients omegaCoefficients = new PIDFCoefficients(0.76, 0, 0.00);
	PIDFCoefficients driveCoefficients = new PIDFCoefficients(0.65, 0, 0);
	public static final double MAX_VELO = 75;
	basedControl distanceController = new basedControl(driveCoefficients, 0, 3, 0.3, 1);
	basedControl omegaController = new basedControl(omegaCoefficients, 0, 3, 0.08, Math.toRadians(1));


	protected double Kp3 = 0.23;
	protected double Kp6 = 7;
	protected double threshold = 2;
	protected robot robot;
	protected double scaler = 0;
	protected double scalerChangeSize = 0.035;

	protected Vector3D output;

	public DifferentialDriveController(robot robot) {
		this.robot = robot;
	}

	/**
	 * calculate robot relative velocities for the system
	 * @param positionSetpoint where the robot should be
	 * @return the motor command to move the robot in robot relative space
	 */
	public Vector3D controllerOutput(Vector3D positionSetpoint) {
		scaler = Range.clip(scaler + scalerChangeSize,0,1);

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
		double driveComponent = driveMultiplier * distanceController.calculate(-distance) / (Kp6 * Range.clip(Math.abs(omegaError), 1, Math.PI * 2));
		double omegaComponent = Math.min(Kp3 * distance, 1) * omegaController.calculate(-omegaError);

		double clippedTurn = Range.clip(omegaComponent, -1, 1);
		return new Vector3D(Range.clip(driveComponent, -1, 1) * scaler, 0, clippedTurn * scaler);

	}

	/**
	 * drive the robot to a desired position on the field using the control law defined in controllerOutput
	 * @param position the desired robot position
	 */
	public boolean driveToPosition(Vector3D position) {
		output = controllerOutput(position);
		robot.driveTrain.robotRelative(output.getX(),output.getAngleRadians());
		drawRobotGreen(position, dashboard.packet);
		return robot.getRobotPose().distanceToPose(position) < threshold + 1.5;
	}

	public void driveToAngle(Vector3D position) {
		position.setX(robot.getRobotPose().getX());
		position.setY(robot.getRobotPose().getY());
		driveToPosition(position);
	}





}
