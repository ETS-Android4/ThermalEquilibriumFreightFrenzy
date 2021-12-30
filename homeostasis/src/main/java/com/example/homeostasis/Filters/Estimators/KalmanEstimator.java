package com.example.homeostasis.Filters.Estimators;

import android.os.Build;

import androidx.annotation.RequiresApi;

import java.util.function.DoubleSupplier;

import com.example.homeostasis.Filters.FilterAlgorithms.KalmanFilter;

/**
 * Using a custom Least Squares Regression + Kalman Filter combination we can accurately reject noise
 * from our signal while attempting to minimize phase lag.
 */
public class KalmanEstimator extends Estimator{

	protected KalmanFilter kalmanFilter;

	/**
	 * @param Q Sensor Covariance
	 * @param R Model Covariance
	 * @param N Number of elements we can hold in our stack.
	 */
	public KalmanEstimator(DoubleSupplier measurement, double Q, double R, int N) {
		super(measurement);
		kalmanFilter = new KalmanFilter(Q,R,N);
	}

	@RequiresApi(api = Build.VERSION_CODES.N)
	@Override
	public double update() {
		return kalmanFilter.update(measurement.getAsDouble());
	}
}
