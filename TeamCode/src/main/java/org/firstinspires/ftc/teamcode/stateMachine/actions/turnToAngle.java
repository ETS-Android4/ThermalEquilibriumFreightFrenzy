package org.firstinspires.ftc.teamcode.stateMachine.actions;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.classicalControl.PIDFCoefficients;
import org.firstinspires.ftc.teamcode.classicalControl.ProfiledPIDController;
import org.firstinspires.ftc.teamcode.geometry.Vector3D;
import org.firstinspires.ftc.teamcode.stateMachine.action;
import org.firstinspires.ftc.teamcode.subsystems.robot;

import static org.firstinspires.ftc.teamcode.utils.utils.AngleWrap;
import static org.firstinspires.ftc.teamcode.utils.utils.normalizeAngleRR;

public class turnToAngle implements action {
	// 1 degree angle tolerance
	double tolerance = Math.toRadians(4);
	double allowedTimeSeconds = 3;
	double error;
	double targetAngle;
	boolean reversed;
	double angleOffset;
	ElapsedTime timeout = new ElapsedTime();
	double max_power = 0.7;
	private org.firstinspires.ftc.teamcode.subsystems.robot robot;
	private boolean isWithinTolerance = false;
	PIDFCoefficients thetaCoefficients = new PIDFCoefficients(0.95,0.01,0);

	private ProfiledPIDController controller = new ProfiledPIDController(thetaCoefficients);



	public turnToAngle(robot robot, double target) {
		this.robot = robot;
		this.reversed = false;
		this.angleOffset = 0;
		this.targetAngle = target;
	}


	@Override
	public void startAction() {

		timeout.reset();
	}

	@Override
	public void runAction() {


		error = AngleWrap(normalizeAngleRR(targetAngle - robot.getRobotPose().getAngleRadians()));

		System.out.println("angle controller turret " + error);
		double power = Range.clip(controller.calculateProfiledOutput(0,-error), -max_power, max_power);

		robot.driveTrain.setMotorPowers(-power, power);

		if ((Math.abs(error) < tolerance && Math.abs(robot.getVelocity().getAngleRadians()) < 0.002) || timeout.seconds() > allowedTimeSeconds) {
			isWithinTolerance = true;
		}

	}

	@Override
	public void stopAction() {
		robot.driveTrain.STOP();
	}

	@Override
	public boolean isActionComplete() {
		return isWithinTolerance;
	}

	@Override
	public boolean isActionPersistant() {
		return true;
	}
}
