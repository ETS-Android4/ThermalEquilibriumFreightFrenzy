package homeostasis2.Controllers;

import homeostasis2.Filters.FilterAlgorithms.LowPassFilter;
import homeostasis2.Parameters.PIDCoefficientsBased;

public class BasedPID extends BasicPID {
	protected PIDCoefficientsBased basedCoefficients;
	protected LowPassFilter filter;
	public BasedPID(PIDCoefficientsBased coefficients) {
		super(coefficients);
		this.basedCoefficients = coefficients;
		this.filter = new LowPassFilter(coefficients.lowPassGain);
	}

	@Override
	protected double calculateDerivative(double error, double dt) {
		derivative = (error - previousError) / dt;
		derivative = this.filter.estimate(derivative);
		return derivative;
	}

	/**
	 * Integral with anti windup methods
	 * @param error the current error of our system
	 * @param dt the time constant.
	 */
	@Override
	protected void integrate(double error, double dt) {
		if (crossOverDetected(error,previousError)) integralSum = 0;
		if (Math.abs(derivative) > basedCoefficients.stabilityThreshold) return;
		integralSum += ((error + previousError) / 2) * dt;
		if (Math.abs(integralSum) > basedCoefficients.maximumIntegralSum) {
			integralSum = Math.signum(integralSum) * basedCoefficients.maximumIntegralSum;
		}
	}

	protected boolean crossOverDetected(double error, double prev) {
		if (error > 0 && prev < 0) return true;
		return error < 0 && prev > 0;
	}

}
