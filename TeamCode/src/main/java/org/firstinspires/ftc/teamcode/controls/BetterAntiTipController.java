package org.firstinspires.ftc.teamcode.controls;

import android.os.Build;

import androidx.annotation.RequiresApi;

import java.util.function.DoubleSupplier;

public class BetterAntiTipController {
	DoubleSupplier robotPitchVelocity;

	protected double reference = Math.toRadians(0);

	BasicPID pid;


	public BetterAntiTipController(DoubleSupplier RobotPitchVelocity) {
		this.robotPitchVelocity = RobotPitchVelocity;
		this.pid = new BasicPID(controllerCoefficients.antiTipCoefficients);
	}

	@RequiresApi(api = Build.VERSION_CODES.N)
	public double antiTip() {
		return pid.calculate(reference, robotPitchVelocity.getAsDouble());
	}

}
