package org.firstinspires.ftc.teamcode.stateMachine.actions;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.basedControl.basedControl;
import org.firstinspires.ftc.teamcode.basedControl.robotCoefficients;
import org.firstinspires.ftc.teamcode.geometry.Vector3D;
import org.firstinspires.ftc.teamcode.stateMachine.action;
import org.firstinspires.ftc.teamcode.subsystems.robot;

import static org.firstinspires.ftc.teamcode.utils.utils.AngleWrap;
import static org.firstinspires.ftc.teamcode.utils.utils.normalizeAngleRR;

public class aimAtPoint implements action {

    // 1 degree angle tolerance
    double tolerance = Math.toRadians(4);
    double allowedTimeSeconds = 3;
    Vector3D targetPosition;
    double error;
    boolean reversed;
    double angleOffset;
    ElapsedTime timeout = new ElapsedTime();
	double max_power = 0.7;
	private final robot robot;
	private boolean isWithinTolerance = false;
	private final boolean reverseAngle;

	private final basedControl controller;


    /**
     * turn the robot to aim at a target point
     *
     * @param robot       robot class
     * @param targetPoint target point to aim at
     * @param reversed    is true if we should aim but at 180 degrees
     * @param angleOffset is the amount we add to the target angle to account for curved shooters and such
     */
    public aimAtPoint(robot robot, Vector3D targetPoint, boolean reversed, double angleOffset, boolean reverseAngle) {
		this.robot = robot;
		this.targetPosition = targetPoint;
		this.reversed = reversed;
		this.angleOffset = angleOffset;
		this.reverseAngle = reverseAngle;
		controller = new basedControl(robotCoefficients.turnCoefficients, 0, 3, 0.004, Math.toRadians(1));

	}
    public aimAtPoint(robot robot, Vector3D targetPoint) {
		this.robot = robot;
		this.targetPosition = targetPoint;
		this.reversed = false;
		this.angleOffset = 0;
		this.reverseAngle = false;
		controller = new basedControl(robotCoefficients.turnCoefficients, 0, 3, 0.004, Math.toRadians(1));

	}
    public aimAtPoint(robot robot, Vector3D targetPoint, boolean reverseAngle) {

		this.robot = robot;
		this.targetPosition = targetPoint;
		this.reversed = false;
		this.angleOffset = 0;
		this.reverseAngle = reverseAngle;
		controller = new basedControl(robotCoefficients.turnCoefficients, 0, 3, 0.004, Math.toRadians(1));

	}

    @Override
    public void startAction() {

        if (reverseAngle) {
            this.targetPosition = this.targetPosition.reflect();
            this.angleOffset = -angleOffset;
        }
        timeout.reset();
    }

    @Override
    public void runAction() {
        com.acmerobotics.roadrunner.geometry.Vector2d difference = targetPosition.toPose2d().vec().minus(robot.getRobotPose().toPose2d().vec());
        double theta;

        if (reversed) {
            theta = (difference.angle() - Math.PI) + angleOffset;
        } else {
            theta = (difference.angle()) + angleOffset;
        }


        error = AngleWrap(normalizeAngleRR(theta - robot.getRobotPose().getAngleRadians()));

        System.out.println("angle controller turret " + error);
		double power = Range.clip(controller.calculateAngle(error), -max_power, max_power);

        robot.driveTrain.setMotorPowers(-power, power);

        if ((Math.abs(error) < tolerance && Math.abs(robot.getVelocity().getAngleRadians()) < 0.002) || timeout.seconds() > allowedTimeSeconds) {
            isWithinTolerance = true;
            System.out.println("aiming towards point: " + targetPosition + " ended with error less than tolerance = " + (Math.abs(error) < tolerance && Math.abs(robot.getVelocity().getAngleRadians()) < 0.002) + " and timeout is = " + (timeout.seconds() > allowedTimeSeconds));
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
    public boolean isActionPersistent() {
        return true;
    }
}
