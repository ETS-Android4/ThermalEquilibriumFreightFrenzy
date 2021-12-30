package homeostasis2;

import homeostasis2.Controllers.Feedback.Controller;
import homeostasis2.Controllers.Feedforward.FeedforwardController;
import homeostasis2.Filters.Estimators.Estimator;

public class SISOsystem {

	public Estimator estimator;
	public Controller feedbackController;
	public FeedforwardController feedforwardController;

	public SISOsystem(Estimator estimator, Controller feedbackController,
					  					   FeedforwardController feedforwardController) {
		this.estimator = estimator;
		this.feedbackController = feedbackController;
		this.feedforwardController = feedforwardController;
	}

	public SISOsystem(Estimator estimator, Controller feedbackController) {
		this.feedforwardController = null;
		this.estimator = estimator;
		this.feedbackController = feedbackController;
	}

	public SISOsystem(Estimator estimator, FeedforwardController feedforwardController) {
		this.estimator = estimator;
		this.feedforwardController = feedforwardController;
	}

	/**
	 * position only control
	 * @param x position control
	 * @return position control output
	 */
	public double update(double x) {
		return update(x,0,0);
	}

	/**
	 * position velocity control
	 * @param x position control
	 * @param v velocity control
	 * @return position, velocity control
	 */
	public double update(double x, double v) {
		return update(x,v,0);
	}

	public double update(double x, double v, double a) {
		double estimate = estimator.update();
		double output = 0;
		if (feedforwardController != null) {
			output += feedforwardController.calculate(x,v,a);
		}
		if (feedbackController != null) {
			output += feedbackController.calculate(x,estimate);
		}
		return output;
	}

}
