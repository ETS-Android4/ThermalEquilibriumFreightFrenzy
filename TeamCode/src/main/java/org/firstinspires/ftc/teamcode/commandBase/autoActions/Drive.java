package org.firstinspires.ftc.teamcode.commandBase.autoActions;

import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.controls.RobustPID;
import org.firstinspires.ftc.teamcode.controls.controllerCoefficients;
import org.firstinspires.ftc.teamcode.classicalControl.PIDFCoefficients;
import org.firstinspires.ftc.teamcode.geometry.Vector3D;
import org.firstinspires.ftc.teamcode.commandBase.action;
import org.firstinspires.ftc.teamcode.subsystems.Dashboard;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

import static org.firstinspires.ftc.teamcode.subsystems.Robot.isCompBot;

/**
 * drive the robot a desired distance or position.
 */
public class Drive implements action {

	protected double targetDistance;
	protected double distance;
	protected boolean isComplete = false;

	protected Robot robot;
	protected Vector3D initialPosition;

	protected RobustPID turnPid;
	protected RobustPID drivePid;
	protected PIDFCoefficients turnCoefficients;
	protected PIDFCoefficients driveCoefficients;
	protected Vector3D targetPosition = null;
	protected double additionalDistance = 0;

	ElapsedTime timer = new ElapsedTime();

	MotionProfile profile;
	public Drive(Robot robot, double targetDistance) {
		this.robot = robot;
		this.targetDistance = targetDistance;
		initializeCoefficients();
	}

	/**
	 * initialize the action with a target position
	 *
	 * Note: this will set the target angle to the initial angle to the target position
	 *
	 * also takes in a scalar parameter, often we don't actually want to travel a full distance
	 * instead we want to travel a percentage of the target distance, the scalar is used for this.
	 *
	 * For example, a scalar of 0.8 will make the robot go 80% of the way to the point
	 *
	 * A scalar of -0.8 will make the robot go in reverse towards the point
	 *
	 *
	 * @param robot target position
	 * @param targetPosition the target position we want to go the distance of
	 * @param scalar the percentage / direction we want to travel in
	 */
	public Drive(Robot robot, Vector3D targetPosition, double scalar) {
		this.robot = robot;
		initializeCoefficients();
		this.targetDistance = scalar;
		this.targetPosition = targetPosition;
	}
	public Drive(Robot robot, Vector3D targetPosition, double scalar, double additionalDistance) {
		this.robot = robot;
		initializeCoefficients();
		this.targetDistance = scalar;
		this.targetPosition = targetPosition;
		this.additionalDistance = additionalDistance * Math.signum(targetDistance);

	}



	@Override
	public void startAction() {
		initialPosition = robot.odometry.subsystemState();
		turnPid = new RobustPID(turnCoefficients, initialPosition.getAngleRadians(), 3, 0.004, Math.toRadians(1));
		drivePid = new RobustPID(driveCoefficients, Math.abs(targetDistance), 3, 0.3, 1);
		initializeTargetAngle();
		this.targetDistance += additionalDistance;
		generateMotionProfile();
		timer = new ElapsedTime();
		timer.reset();
	}

	@Override
	public void runAction() {

		distance = initialPosition.distanceToPose(robot.odometry.subsystemState());

		double targetDistProfile = profile.get(timer.seconds()).getX();
		drivePid.setReference(targetDistProfile);

		double drive = drivePid.calculate(distance) * Math.signum(targetDistance);
		double turn = turnPid.calculateLinearAngle(robot.odometry.subsystemState().getAngleRadians());

		robot.driveTrain.robotRelative(drive, turn);
		isComplete = ((drivePid.isComplete() || drivePid.isVeryStable()) || drivePid.isBasicallyStopped()) && profile.duration() < timer.seconds();


		Dashboard.packet.put("distance traveled", distance);
		Dashboard.packet.put("target distance", targetDistProfile);

	}

	@Override
	public void stopAction() {

		robot.driveTrain.STOP();

	}

	@Override
	public boolean isActionComplete() {
		return isComplete;
	}

	@Override
	public boolean isActionPersistent() {
		return true;
	}


	public void generateMotionProfile() {
		if (isCompBot) {
			profile = MotionProfileGenerator.generateSimpleMotionProfile(
					new MotionState(0,0,0),
					new MotionState(Math.abs(this.targetDistance),0,0),
					controllerCoefficients.compBotVelocity,
					controllerCoefficients.compBotAcceleration,
					controllerCoefficients.compBotJerk);
		} else {
			profile = MotionProfileGenerator.generateSimpleMotionProfile(
					new MotionState(0,0,0),
					new MotionState(Math.abs(this.targetDistance),0,0),
					controllerCoefficients.protoBotVelocity,
					controllerCoefficients.protoBotAcceleration,
					controllerCoefficients.protoBotJerk);
		}
	}

	public void initializeCoefficients() {
		if (isCompBot) {
			turnCoefficients = controllerCoefficients.compBotDriveCorrect;
			driveCoefficients = controllerCoefficients.compBotDrive;
		} else {
			turnCoefficients = controllerCoefficients.protoBotDriveCorrect;
			driveCoefficients = controllerCoefficients.protoBotDrive;
		}
	}

	public void initializeTargetAngle() {
		if (targetPosition != null) {

			Vector3D positionError = targetPosition.getError(initialPosition);
			double omega1 = Math.atan2(positionError.getY(),positionError.getX());
			double omega2 = Math.atan2(positionError.getY(),positionError.getX()) - Math.toRadians(180);
			this.targetDistance = Math.abs(this.targetPosition.distanceToPose(initialPosition)) * this.targetDistance;
			drivePid.setReference(Math.abs(this.targetDistance));

			if (this.targetDistance > 0) {
				turnPid.setReference(omega1);
			}
			else {
				turnPid.setReference(omega2);
			}
		}
	}
}
