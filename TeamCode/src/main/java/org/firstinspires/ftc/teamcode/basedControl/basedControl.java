package org.firstinspires.ftc.teamcode.basedControl;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.classicalControl.PIDFCoefficients;
import org.firstinspires.ftc.teamcode.utils.RingBuffer;

import static org.firstinspires.ftc.teamcode.utils.utils.normalizedHeadingError;

public class basedControl {

	// coefficients of the controller
	protected PIDFCoefficients coefficients;

	// elapsed timer used for integral and derivative calculations
	protected ElapsedTime timer;

	private final RingBuffer<PIDState> derivativeBuffer; // good morning bfr <3

	// process error
	protected double error = 0;
	// previous error
	protected double lastError = 0;
	// cumulative error sum over time
	protected double integral_sum = 0;
	// error rate of change
	protected double derivative = 0;
	// output that we send to our plant
	protected double output = 0;
	// delta time between updates
	protected double dt = 0;
	// previous time stamp (seconds)
	protected double lastTime;
	// current time stamp (seconds)
	protected double time;
	// where our plants state should be
	protected double reference;
	// how long the RingBuffers length should store data for
	protected int bufferLength;
	// exit tolerance
	protected double exitTolerance;
	// if the derivative is above this, we are not considering the system stable
	protected double stability_threshold;

	/**
	 * construct PID with buffer length and stability threshold
	 *
	 * @param coefficients        pid coefficients
	 * @param reference           the target state our system should be in
	 * @param bufferLength        how long our buffer should be
	 * @param stability_threshold how stable our derivative needs to be inorder to stop
	 */
	public basedControl(PIDFCoefficients coefficients, double reference, int bufferLength, double stability_threshold, double exitTolerance) {
		this.reference = reference;
		this.bufferLength = bufferLength;
		this.stability_threshold = stability_threshold;
		this.coefficients = coefficients;
		this.exitTolerance = exitTolerance;
		timer = new ElapsedTime();
		lastTime = timer.time();
		PIDState PIDFData = new PIDState(lastError, lastTime);
		derivativeBuffer = new RingBuffer<>(bufferLength, PIDFData);
	}

	/**
	 * perform general PID calculations and operations
	 */
	protected void baseCalculate() {
		updateTime();
		calculateDerivative();
		calculateIntegral();

		output = (error * coefficients.Kp) +
				(reference * coefficients.Kf) +
				(derivative * coefficients.Kd) +
				(integral_sum * coefficients.Ki);

		lastError = error;
	}


	/**
	 * calculate pid output with normal linear values
	 * @param state current measurement of our system
	 * @return input to the plant
	 */
	public double calculate(double state) {
		calculateErrorNormal(state);
		baseCalculate();
		return output;
	}


	/**
	 * calculate pid output with angle values
	 * @param state current measurement of our system (radians)
	 * @return input to the plant
	 */
	public double calculateAngle(double state) {
		calculateErrorAngle(state);
		baseCalculate();
		output = opAngleController() + (derivative * coefficients.Kd) + (integral_sum * coefficients.Ki);
		return output;
	}
	/**
	 * calculate pid output with angle values
	 * @param state current measurement of our system (radians)
	 * @return input to the plant
	 */
	public double calculateLinearAngle(double state) {
		calculateErrorAngle(state);
		baseCalculate();
		return output;
	}

	/**
	 * calculate derivative using the buffer to smooth the data
	 */
	protected void calculateDerivative() {
		PIDState bufferedPoint = derivativeBuffer.insert(new PIDState(error, time));

		derivative = (error - bufferedPoint.error) / (time - bufferedPoint.timeStamp);
	}

	/**
	 * calculate integral sum given our system is stable
	 */
	protected void calculateIntegral() {
		double avg = (error + lastError) / 2;

		if (shouldIntegrate()) {
			integral_sum += avg * dt;
		}
		antiWindup();

	}

	/**
	 * assess if our system is stable by checking the derivative
	 * @return true if stable, false if not
	 */
	public boolean isStable() {
		return Math.abs(stability_threshold) > Math.abs(derivative);
	}

	/**
	 * conditions for if we should integrate
	 * @return true if we should integrate
	 */
	public boolean shouldIntegrate() {
		return isStable();
	}

	/**
	 * update time step
	 */
	protected void updateTime() {
		time = timer.seconds();
		dt = time - lastTime;
		lastTime = time;
	}

	/**
	 * calculate linear error
	 * @param state systems state
	 */
	protected void calculateErrorNormal(double state) {
		error = reference - state;
	}

	/**
	 * calculate angle error
	 * @param state systems state
	 */
	protected void calculateErrorAngle(double state) {
		error = normalizedHeadingError(reference, state);
	}

	/**
	 * return the current plant error
	 * @return error
	 */
	public double getError() {
		return error;
	}
	
	protected double opAngleController() {
		double hysteresisAmount = 0.96;
		if (error > 0) {
			return Math.pow(coefficients.Kp, error) - hysteresisAmount;
		}
		return -Math.pow(coefficients.Kp, -error) + hysteresisAmount;
	}

	public boolean isComplete() {
		return Math.abs(error) < exitTolerance && isStable();
	}

	public void antiWindup() {
		integral_sum = Range.clip(integral_sum, -0.5, 0.5);
		if ((lastError > 0 && error < 0) || ((lastError < 0 && error > 0))) {
			integral_sum = 0;
		}
	}
}
