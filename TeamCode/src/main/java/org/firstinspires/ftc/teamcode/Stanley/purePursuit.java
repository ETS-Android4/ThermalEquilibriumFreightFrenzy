package org.firstinspires.ftc.teamcode.Stanley;

import org.firstinspires.ftc.teamcode.geometry.Vector3D;
import org.firstinspires.ftc.teamcode.subsystems.robot;

public class purePursuit {

	robot robot;

	protected double A = 0;
	protected double B = 0;
	protected double C = 0;

	public purePursuit(robot robot) {
		this.robot = robot;
	}

	public void driveToPosition(Vector3D target) {

	}

	public void getLine(Vector3D startPosition, Vector3D endPosition) {
		double m = (endPosition.getY() - startPosition.getY()) / (endPosition.getX() - startPosition.getX());
		double b = -m * endPosition.getX() + endPosition.getY();

	}

}
