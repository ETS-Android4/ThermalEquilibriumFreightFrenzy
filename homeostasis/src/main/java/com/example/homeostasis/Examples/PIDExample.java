package com.example.homeostasis.Examples;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.function.DoubleSupplier;

import com.example.homeostasis.Controllers.Feedback.PIDEx;
import com.example.homeostasis.Controllers.Feedforward.FeedforwardEx;
import com.example.homeostasis.Filters.Estimators.Estimator;
import com.example.homeostasis.Filters.Estimators.LowPassEstimator;
import com.example.homeostasis.Parameters.FeedforwardCoefficientsEx;
import com.example.homeostasis.Parameters.PIDCoefficients;
import com.example.homeostasis.Parameters.PIDCoefficientsEx;
import com.example.homeostasis.Systems.BasicSystem;

public class PIDExample extends LinearOpMode {
	PIDCoefficients coefficients = new PIDCoefficients(1,0,0);
	protected DcMotorEx motor;

	@RequiresApi(api = Build.VERSION_CODES.N)
	@Override
	public void runOpMode() throws InterruptedException {
		motor = hardwareMap.get(DcMotorEx.class, "motor");

		PIDCoefficientsEx coefficients = new PIDCoefficientsEx(1,
																		0,
																		0,
																		0.1,
																		3,
																		3);


		FeedforwardCoefficientsEx fCoefficients = new FeedforwardCoefficientsEx(1,1,0,0,1);


		PIDEx PID = new PIDEx(coefficients);
		FeedforwardEx feedforward = new FeedforwardEx(fCoefficients);
		Estimator lowPassFilter = new LowPassEstimator(new DoubleSupplier() {
			@Override
			public double getAsDouble() {
				return motor.getCurrentPosition();
			}
		}, 0.9);

		BasicSystem sys = new BasicSystem(lowPassFilter,PID,feedforward);

		waitForStart();
		while (opModeIsActive()) {

		}
	}


}
