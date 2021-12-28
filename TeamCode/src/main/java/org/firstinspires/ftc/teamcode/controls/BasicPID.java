package org.firstinspires.ftc.teamcode.controls;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.classicalControl.PIDFCoefficients;


/**
 * A basic PID controller that is nice when we just need something to work.
 */
public class BasicPID {
	protected PIDFCoefficients coefficients;
	private double previousError = 0;
	private double integralSum = 0;
	private boolean hasRun = false;

	private ElapsedTime timer = new ElapsedTime();

	public BasicPID(PIDFCoefficients coefficients) {
		this.coefficients = coefficients;
	}

	public double calculate(double reference, double state) {
		double error = reference - state;
		double dt = getDt();
		double derivative = (error - previousError) / dt;
		integralSum += ( (error + previousError) / 2) * dt;
		previousError = error;
		return coefficients.Kp * error + coefficients.Ki * integralSum + coefficients.Kd * derivative;
	}

	public double calculate(double error) {
		return calculate(0, -error);
	}

	private double getDt() {
		if (!hasRun) {
			hasRun = true;
			timer.reset();
		}
		double dt = timer.seconds();
		timer.reset();
		return dt;
	}


}
