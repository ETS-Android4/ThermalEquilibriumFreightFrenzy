package org.firstinspires.ftc.teamcode.classicalControl;

import org.firstinspires.ftc.teamcode.Filter.LowPassFilter;
import org.firstinspires.ftc.teamcode.subsystems.Dashboard;

import homeostasis.systems.DcMotorPlant;

public class velocityControl {

	DcMotorPlant motor;

	protected PIDFCoefficients coefficients = new PIDFCoefficients(0,0,0,0,0.0003767);
	protected NonlinearPID pidController = new NonlinearPID(coefficients);
	protected LowPassFilter rateLimiter = new LowPassFilter(0.3);

	public void initializeControllerSettings() {

	}
	public velocityControl(DcMotorPlant motor, PIDFCoefficients coefficients) {
		this.motor = motor;
		this.coefficients = coefficients;
		this.pidController = new NonlinearPID(coefficients);
		initializeControllerSettings();
	}

	public velocityControl(DcMotorPlant motor) {
		this.motor = motor;
		initializeControllerSettings();
	}

	public void voltageCorrectedControl(double targetVelocity, double targetVoltage) {
		double state = motor.getState().getVelocity();
		double target = rateLimiter.updateEstimate(targetVelocity);
		double output = pidController.calculateOutputPFOnly(target, state);
		Dashboard.packet.put("measured velo", state);
		Dashboard.packet.put("rate limited velo", target);
		Dashboard.packet.put("target velo", targetVelocity);
		motor.input(output * (13.8 / targetVoltage));
	}

	public void controlMotor(double targetVelocity) {

		double state = motor.getState().getVelocity();
		double output = pidController.calculateOutput(targetVelocity,state);

		motor.input(output);
	}


	public void setCoefficients(PIDFCoefficients coefficients) {
		this.coefficients = coefficients;
		this.pidController = new NonlinearPID(coefficients);
	}
}
