package homeostasis2.Filters.Estimators;

import android.os.Build;

import androidx.annotation.RequiresApi;

import java.util.function.DoubleSupplier;


/**
 * When you have good sensors.
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

	@RequiresApi(api = Build.VERSION_CODES.N)
	@Override
	public double update() {
		return measurement.getAsDouble();
	}
}
