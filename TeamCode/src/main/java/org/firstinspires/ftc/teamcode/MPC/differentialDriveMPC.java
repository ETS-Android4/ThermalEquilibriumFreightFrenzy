package org.firstinspires.ftc.teamcode.MPC;

import org.firstinspires.ftc.teamcode.geometry.Vector3D;
import org.firstinspires.ftc.teamcode.subsystems.robot;

public class differentialDriveMPC {

	protected robot robot;

	public differentialDriveMPC(robot robot) {
		this.robot = robot;
	}

	/**
	 * drive the robot to target position using mpc
	 * @param target the target position
	 * @return the drive command, x is forward, radians is turn power
	 */
	public Vector3D driveCommand(Vector3D target) {
		return new Vector3D(1,0,0);
	}


	/**
	 * given a model,
	 * @param driveCommand
	 * @return
	 */
	public Vector3D[] updateModel(Vector3D driveCommand) {
		return new Vector3D[] {new Vector3D(), new Vector3D()};
	}

}
