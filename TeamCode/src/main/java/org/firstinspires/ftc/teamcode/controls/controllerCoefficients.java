package org.firstinspires.ftc.teamcode.controls;

import org.firstinspires.ftc.teamcode.classicalControl.PIDFCoefficients;

public class controllerCoefficients {

	public static final double compBotVelocity = 70;
	public static final double compBotAcceleration = 90;
	public static final double compBotJerk = 130;

	public static final double protoBotVelocity = 100;
	public static final double protoBotAcceleration = 200;
	public static final double protoBotJerk = 250;

	// controller coefficients for competition 10wd
	public static final PIDFCoefficients compBotTurn = new PIDFCoefficients(1, 0.8, 0.09,0,0,0.1); // 0.2785000000000002
	public static final PIDFCoefficients compBotDrive = new PIDFCoefficients(0.071, 0.001, 0.01,0,0,0.04350000000000015);
	public static final PIDFCoefficients compBotDriveCorrect = compBotTurn.noFeedforward();
	public static final PIDFCoefficients compBotDriveVelo = new PIDFCoefficients(0.02,0.03,0,0,1/compBotVelocity,0);
	// controller coefficients for off season 6wd
	public static final PIDFCoefficients protoBotTurn = PIDFCoefficients.JaRule(0.8);
	public static final PIDFCoefficients protoBotDrive = new PIDFCoefficients(0,0,0);//new PIDFCoefficients(0.18238, 0, 0.00993, 0, 0,0.04770000000000017);
	public static final PIDFCoefficients protoBotDriveCorrect = protoBotTurn.noFeedforward();
	public static final PIDFCoefficients protoBotDriveVelo = new PIDFCoefficients(0.02,0.03,0,0,1/protoBotVelocity,0);

	// controller coefficients for linear slide subsystem
	public static final PIDFCoefficients slideCoefficients = new PIDFCoefficients(0.007, 0.08, 0);

	public static final BangBangParameters protoBotAntiTip = new BangBangParameters(1/1.5 , Math.toRadians(5),0,0);





}
