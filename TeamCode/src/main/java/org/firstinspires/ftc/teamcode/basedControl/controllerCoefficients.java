package org.firstinspires.ftc.teamcode.basedControl;

import org.firstinspires.ftc.teamcode.classicalControl.PIDFCoefficients;

public class controllerCoefficients {

	// controller coefficients for competition 10wd
	public static final PIDFCoefficients compBotTurn = new PIDFCoefficients(2, 0.2, 0.2);
	public static final PIDFCoefficients compBotDrive = new PIDFCoefficients(0.1, 0, 0.01);
	public static final PIDFCoefficients compBotDriveCorrect = new PIDFCoefficients(0.9, 0, 0.1);

	// controller coefficients for off season 6wd
	public static final PIDFCoefficients protoBotTurn = new PIDFCoefficients(0.49, 0, 0.055, 0, 0, 0.05000000000000004);
	public static final PIDFCoefficients protoBotDrive = new PIDFCoefficients(0.1, 0, 0.01, 0, 0, 0.048500000000000036);
	public static final PIDFCoefficients protoBotDriveCorrect = new PIDFCoefficients(0.9, 0, 0.1);

	// controller coefficients for linear slide subsystem
	public static final PIDFCoefficients slideCoefficients = new PIDFCoefficients(0.007, 0.08, 0);


}
