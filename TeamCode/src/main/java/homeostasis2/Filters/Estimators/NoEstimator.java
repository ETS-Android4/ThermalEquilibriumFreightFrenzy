package homeostasis2.Filters.Estimators;

import java.util.function.DoubleSupplier;


/**
 * Estimator that does no processing at all!
 *
 * I love oop.
 */
public class NoEstimator extends Estimator{
	/**
	 * Set up Double Supplier for recurring measurement obtainment.
	 *
	 * @param measurement measurement we want to obtain.
	 */
	public NoEstimator(DoubleSupplier measurement) {
		super(measurement);
	}

	@Override
	public double update() {
		return measurement.getAsDouble();
	}
}
