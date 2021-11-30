package org.firstinspires.ftc.teamcode.MotionProfiling;

import com.qualcomm.robotcore.util.ElapsedTime;

public class EasyOnlineMotionProfile {

	double maxAccel;
	double maxVelo;
	double lastVelocity = 0;
	double velocity = 0;
	double position = 0;
	ElapsedTime timer = new ElapsedTime();

	public EasyOnlineMotionProfile(double maxVelo, double maxAccel) {
		this.maxVelo = maxVelo;
		this.maxAccel = maxAccel;
	}

	public void updateProfile(double error) {
		double dt = timer.seconds();
		timer.reset();
		double maxVelToStop = Math.sqrt(2.0 * maxAccel * error);
		double maxVelFromLast = lastVelocity + maxAccel * dt;
		velocity = Math.min(maxVelFromLast, Math.min(maxVelToStop, maxVelo));
		double acceleration = (velocity - lastVelocity) / dt;
		lastVelocity = velocity;
	}
	public double getVelocity() {
		return velocity;
	}

	public double getPosition() {
		return position;
	}
}
