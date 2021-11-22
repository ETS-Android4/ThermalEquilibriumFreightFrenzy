package org.firstinspires.ftc.teamcode.basedControl;

import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;

import org.firstinspires.ftc.teamcode.classicalControl.PIDFCoefficients;

public class controllerCoefficients {

	// controller coefficients for competition 10wd
	public static final PIDFCoefficients compBotTurn = new PIDFCoefficients(0.8, 0.01, 0.05,0,0,0.1805 * 1.2);
	public static final PIDFCoefficients compBotDrive = new PIDFCoefficients(0.071, 0.001, 0.01);
	public static final PIDFCoefficients compBotDriveCorrect = compBotTurn.noFeedforward();

	// controller coefficients for off season 6wd
	public static final PIDFCoefficients protoBotTurn = new PIDFCoefficients(0.59, 0.16, 0.01, 0, 0, 0.09250000000000004);
	public static final PIDFCoefficients protoBotDrive = new PIDFCoefficients(0.08238, 0, 0.00993, 0, 0,0.03630000000000011);
	public static final PIDFCoefficients protoBotDriveCorrect = protoBotTurn.noFeedforward();

	// controller coefficients for linear slide subsystem
	public static final PIDFCoefficients slideCoefficients = new PIDFCoefficients(0.007, 0.08, 0);


	public static final double compBotVelocity = 100;
	public static final double compBotAcceleration = 100;
	public static final double compBotJerk = 250;
	public static final double protoBotVelocity = 100;
	public static final double protoBotAcceleration = 100;
	public static final double protoBotJerk = 250;



}
