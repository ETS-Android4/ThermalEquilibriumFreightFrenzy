package org.firstinspires.ftc.teamcode.commandBase.autoActions.DrivetrainControl;

import org.firstinspires.ftc.teamcode.Geometry.Vector3D;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

public class DriveToIntake extends DriveToPosition {
	public DriveToIntake(Robot robot, Vector3D referencePose, double cutOffTime, boolean useMaxAccel) {
		super(robot, referencePose, cutOffTime, useMaxAccel);
	}
}
