package homeostasis2.Parameters;

public class PIDCoefficientsBased extends PIDCoefficients{
	public double maximumIntegralSum;
	public double stabilityThreshold;
	public double ringBufferLength;
	public double lowPassGain;

	/**
	 * @param Kp proportional term
	 * @param Ki integral term
	 * @param Kd derivative term
	 * @param maximumIntegralSum the maximum our integral sum can go to
	 *                              (this * Kp should be less than the maximum output of your system)
	 * @param stabilityThreshold the maximum our derivative can be before we integrate.
	 *                           This ensures we have better stability
	 * @param ringBufferLength The size of our ring buffer.  We use a ring buffer to smooth our derivative.
	 *                         Longer ring buffer lengths have less noise but use more stale values.
	 */
	public PIDCoefficientsBased(double Kp, double Ki, double Kd,
								double maximumIntegralSum,
								double stabilityThreshold,
								double ringBufferLength,
								double lowPassGain) {
		super(Kp, Ki, Kd);
		this.maximumIntegralSum = maximumIntegralSum;
		this.stabilityThreshold = stabilityThreshold;
		this.ringBufferLength = ringBufferLength;
		this.lowPassGain = lowPassGain;
	}
}
