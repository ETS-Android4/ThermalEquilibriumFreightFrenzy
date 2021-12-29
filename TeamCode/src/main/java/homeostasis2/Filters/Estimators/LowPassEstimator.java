package homeostasis2.Filters.Estimators;

import android.os.Build;

import androidx.annotation.RequiresApi;

import java.util.function.DoubleSupplier;

import homeostasis2.Filters.FilterAlgorithms.LowPassFilter;

public class LowPassEstimator extends Estimator{

	protected LowPassFilter filter;

	/**
	 * Set up Double Supplier for recurring measurement obtainment.
	 *
	 * @param measurement measurement we want to obtain.
	 */
	public LowPassEstimator(DoubleSupplier measurement, double LowPassGain) {
		super(measurement);
		filter = new LowPassFilter(LowPassGain);
	}

	@RequiresApi(api = Build.VERSION_CODES.N)
	@Override
	public double update() {
		return filter.estimate(measurement.getAsDouble());
	}
}
