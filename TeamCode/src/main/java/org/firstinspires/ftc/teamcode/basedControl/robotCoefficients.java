package org.firstinspires.ftc.teamcode.basedControl;

import org.firstinspires.ftc.teamcode.classicalControl.PIDFCoefficients;

public class robotCoefficients {

	public static final PIDFCoefficients turnCoefficients = new PIDFCoefficients(4.3, 0.08, 0.03);
	public static final PIDFCoefficients driveCoefficients = new PIDFCoefficients(0.1, 0, 0.01);

}
