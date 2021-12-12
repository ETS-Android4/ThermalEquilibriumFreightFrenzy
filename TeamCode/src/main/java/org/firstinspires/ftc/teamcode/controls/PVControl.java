package org.firstinspires.ftc.teamcode.controls;

import homeostasis.utils.State;

public class PVControl {

	protected PVParams coefficients;
	protected double previousFeedbackOutput = 0;
	protected boolean isProcessComplete = false;

	public PVControl(PVParams coefficients) {
		this.coefficients = coefficients;
	}

	public double calculate(double positionReference,
							double velocityReference, double positionState,  double velocityState) {

		return calculate(new State(positionReference,velocityReference),
						 new State(positionState,velocityState));

	}

	public double calculate(State reference,State state) {
		State error = reference.stateError(state);
		double feedback = calculateFeedback(error);
		double feedforward = calculateFeedforward(reference.getVelocity(), feedback);
		isProcessComplete = isCompleteCalc(error);
		return feedback + feedforward;
	}

	protected double calculateFeedback(State error) {
		return error.getPosition() * coefficients.getKp()
			 + error.getVelocity() * coefficients.getKpV();
	}

	protected double calculateFeedforward(double referenceVelocity, double feedbackOutput) {
		double out = referenceVelocity * coefficients.getKv()
				   + Math.signum(feedbackOutput) * coefficients.getKs()
				   + previousFeedbackOutput * coefficients.getAff();
		previousFeedbackOutput = feedbackOutput;
		return out;
	}

	public boolean isCompleteCalc(State error) {
		return Math.abs(error.getVelocity()) < coefficients.getCutOffVelo()
				|| Math.abs(error.getPosition()) < coefficients.getCutOffPos();
	}

	public boolean isProcessComplete() {
		return isProcessComplete;
	}
}

