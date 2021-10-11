package homeostasis.Filters;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;

@Config
public class angleKalmanFilter {

	protected double x;
	protected double p;
	protected double x_previous;
	protected double p_previous;
	public static double Q = 9.504936835155863;
	public static double R = 7.815049368351558;
	protected double currentModel;
	protected double previousModel;
	protected double currentSensor2;
	protected double previousSensor2;

	protected double A = 1;
	protected double C = 1;
	protected double H = 1;
	protected double I = 1;
	protected double kalmanGain;

	ElapsedTime timer = new ElapsedTime();

	public angleKalmanFilter(double initialCondition) {
		this.x = initialCondition;
		this.x_previous = initialCondition;
		this.p = 0;
		this.p_previous = 0;
		this.currentModel = initialCondition;
		this.previousModel = initialCondition;
		this.currentSensor2 = initialCondition;
		this.previousSensor2 = initialCondition;

	}

	/**
	 * update the odometry measurement from our sensor1 and gyro angle
	 *
	 * @param model primary angle sensor
	 * @param sensor2 most stable angle sensor that we are converging on over time
	 * @return estimated angle from kalman filter
	 */
	public double updateKalmanEstimate(double model, double sensor2) {


		currentModel = model;
		currentSensor2 = sensor2;


		double u = normalizeRadians(currentModel - previousModel);

		double angularVelocity = u / timer.seconds();
		System.out.println("robot angular velocity is " + angularVelocity);
		timer.reset();;

		p = p_previous + Q;

		if (false) {
			kalmanGain = 0;
			x = normalizeRadians(x + currentSensor2);
		} else {
			x = x_previous + u;
			kalmanGain = p * H * (1 / (H * p * H + R));
			x = normalizeRadians(x + kalmanGain * normalizeRadians((currentSensor2) - H * normalizeRadians(x - x_previous)));
		}

		x_previous = x;

		previousModel = currentModel;
		previousSensor2 = currentSensor2;

		p = I - kalmanGain * H * p;

		p_previous = p;

		return x;
	}


}
