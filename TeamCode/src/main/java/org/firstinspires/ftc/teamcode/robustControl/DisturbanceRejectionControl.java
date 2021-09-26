package org.firstinspires.ftc.teamcode.robustControl;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 *
 * Modeled after pid control but with numerous performance enhancements to ensure robustness
 *
 * Nonlinear feedback controller that uses a kalman filter as it's derivative and a bilinear transform integral
 * also contains feedforward for trajectory traversal performance increases
 *
 * In addition to an integral we use adapative feedforward control to ensure steady state performance
 */
public class DisturbanceRejectionControl {

	protected ADRCParameters coefficients;
	private double previousError = 0;
	private double integral = 0;
	private double adaptiveFeedforward = 0;
	ElapsedTime timer;

	/**
	 * initialize active disturbance rejection control parameters
	 * @param coefficients coefficients
	 */
	public DisturbanceRejectionControl(ADRCParameters coefficients) {
		this.coefficients = coefficients;
		timer = new ElapsedTime();
	}


	/**
	 * calculate feedback using active disturbance rejection control
	 * @param setpoint this is where our system needs to be
	 * @param position this is where our system is
	 * @param velocityEstimate this is the velocity estimate of our system, used as a derivative
	 *                         using a kalman filtered derivative enables disturbance rejection properties
	 *
	 *                         In addition we use a second order integral to improve stability
	 *                         This method is called the bilinear transform and any continuous control law
	 *                         that is converted to discrete time using the bilinear transform
	 *                         maps the poles from the left half plane to the z domain unit circle
	 *
	 *                         Therefore any and all controllers designed using the bilinear transform
	 *                         will remain stable despite discretization
	 *
	 * @return active disturbance rejection control
	 */
	public double feedback(double setpoint, double position, double velocityEstimate) {


		double error = setpoint - position;

		// bilinear transform integral
		integral += ((error + previousError) / 2) * timer.seconds();

		double proportionalTerm = error * coefficients.getKp();
		double integralTerm = integral * coefficients.getKi();
		double derivativeTerm = -velocityEstimate * coefficients.getKd();

		previousError = error;

		double output = proportionalTerm + integralTerm + derivativeTerm;
		double outputAFF = output + (adaptiveFeedforward * coefficients.getAff());
		adaptiveFeedforward = output;

		return outputAFF;
	}


	/**
	 * calculate velocity feedforward
	 * @param velocityReference how fast we would like to travel
	 * @param accelerationReference how fast we would like to accelerate
	 * @return the feedforward approximation
	 */
	public double feedforward(double velocityReference, double accelerationReference) {
		return velocityReference * coefficients.getKv() + accelerationReference * coefficients.getKa();
	}

	/**
	 * controller that takes advantage of both feedforward and feedback for trajectory traversing
	 * @param setpoint the position we would like to be at
	 * @param position the current system position
	 * @param velocityEstimate the current estimated velocity (preferably from state observer)
	 * @param velocityReference how fast we would like to travel
	 * @param accelerationReference how fast we would like to accelerate
	 * @return actuator command
	 */
	public double dripControl(double setpoint, double position, double velocityEstimate,
							  double velocityReference, double accelerationReference) {
		return feedback(setpoint, position, velocityEstimate)
				+ feedforward(velocityReference, accelerationReference);
	}
}
