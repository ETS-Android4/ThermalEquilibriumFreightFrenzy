package org.firstinspires.ftc.teamcode.robustControl;

public class ADRCParameters {

	private final double kp;
	private final double ki;
	private final double kd;
	private final double kpV;
	private final double kv;
	private final double ka;
	private final double aff; // adaptive feedforward


	public ADRCParameters(double Kp, double Ki, double Kd, double aff, double KpV, double Kv, double Ka) {
		kp = Kp;
		ki = Ki;
		kd = Kd;
		this.aff = aff;
		kpV = KpV;
		kv = Kv;
		ka = Ka;
	}

	public double getKpV() {
		return kpV;
	}

	public double getKv() {
		return kv;
	}

	public double getKa() {
		return ka;
	}

	public double getKp() {
		return kp;
	}

	public double getKi() {
		return ki;
	}

	public double getKd() {
		return kd;
	}

	public double getAff() {
		return aff;
	}
}
