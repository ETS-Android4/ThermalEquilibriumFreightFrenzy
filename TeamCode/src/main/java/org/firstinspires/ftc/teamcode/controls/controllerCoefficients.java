package org.firstinspires.ftc.teamcode.controls;

import org.firstinspires.ftc.teamcode.classicalControl.PIDFCoefficients;

public class controllerCoefficients {

	// controller coefficients for competition 10wd
	public static final PIDFCoefficients compBotTurn = new PIDFCoefficients(1, 0.8, 0.09,0,0,0.1); // 0.2785000000000002
	public static final PIDFCoefficients compBotDrive = new PIDFCoefficients(0.071, 0.001, 0.01,0,0,0.04350000000000015);
	public static final PIDFCoefficients compBotDriveCorrect = compBotTurn.noFeedforward();

	// controller coefficients for off season 6wd
	public static final PIDFCoefficients protoBotTurn = new PIDFCoefficients(0.59, 0.16, 0.09, 0, 0, 0.0580);
	public static final PIDFCoefficients protoBotDrive = new PIDFCoefficients(0.18238, 0, 0.00993, 0, 0,0.04770000000000017);
	public static final PIDFCoefficients protoBotDriveCorrect = protoBotTurn.noFeedforward();

	// controller coefficients for linear slide subsystem
	public static final PIDFCoefficients slideCoefficients = new PIDFCoefficients(0.007, 0.08, 0);

	public static final BangBangParameters protoBotAntiTip = new BangBangParameters(0.5, Math.toRadians(5),0.09,0);


	public static final double compBotVelocity = 70;
	public static final double compBotAcceleration = 90;
	public static final double compBotJerk = 130;

	public static final double protoBotVelocity = 100;
	public static final double protoBotAcceleration = 100;
	public static final double protoBotJerk = 250;



}
