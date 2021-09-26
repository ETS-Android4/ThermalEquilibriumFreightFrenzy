package org.firstinspires.ftc.teamcode.filter;

import org.firstinspires.ftc.teamcode.geometry.Vector3D;

import homeostasis.Filters.SISOKalmanFilter;

public class positionKalmanFilter {


	SISOKalmanFilter xEstimator;
	SISOKalmanFilter yEstimator;

	Vector3D positionEstimate;
	Vector3D positionEstimateDelta;

	public positionKalmanFilter(Vector3D initialPosition) {
		double q = 3;
		double r = 4;
		xEstimator = new SISOKalmanFilter();
		xEstimator.setCovariance(q,r);
		yEstimator = new SISOKalmanFilter();
		yEstimator.setCovariance(q,r);
		this.positionEstimate = initialPosition;
	}


	/**
	 * get new deltas from kalman filter
	 * @param model model delta
	 * @param sensor sensor delta
	 * @return filtered delta
	 */
	public Vector3D updateKalmanFilter(Vector3D model, Vector3D sensor) {
		double xEstimate = xEstimator.updateKalmanFilter(sensor.getX(),model.getX());
		double yEstimate = yEstimator.updateKalmanFilter(sensor.getY(),model.getY());
		positionEstimateDelta = new Vector3D(xEstimate, yEstimate, 0);

		return positionEstimateDelta;
	}
	public Vector3D updatePoseEstimate(Vector3D model, Vector3D sensor) {
		positionEstimate = positionEstimate.add(updateKalmanFilter(model, sensor));
		positionEstimate.setAngleRad(model.getAngleRadians());
		return positionEstimate;
	}

	public Vector3D getPositionEstimate() {
		return positionEstimate;
	}

	public Vector3D getPositionEstimateDelta() {
		return positionEstimateDelta;
	}
}
