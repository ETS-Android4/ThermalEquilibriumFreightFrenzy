package homeostasis2.Filters.Estimators;

import java.util.function.DoubleSupplier;

public abstract class Estimator {

	public DoubleSupplier measurement;

	/**
	 * Set up Double Supplier for recurring measurement obtainment.
	 * @param measurement measurement we want to obtain.
	 */
	public Estimator(DoubleSupplier measurement) {
		this.measurement = measurement;
	}

	public abstract double update();

}
