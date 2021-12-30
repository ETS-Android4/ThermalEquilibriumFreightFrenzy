package com.example.homeostasis.Filters.Estimators;

import android.os.Build;

import androidx.annotation.RequiresApi;

import java.util.function.DoubleSupplier;

import com.example.homeostasis.Filters.FilterAlgorithms.LowPassFilter;


public class LowPassEstimator extends Estimator{

	protected LowPassFilter filter;

	/**
	 * Set up Double Supplier for recurring measurement obtainment.
	 *
	 * Uses a low pass filter to estimate the systems state.
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
