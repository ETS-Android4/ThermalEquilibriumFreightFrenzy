package homeostasis2.Examples;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.function.DoubleSupplier;

import homeostasis2.Controllers.Feedback.PIDEx;
import homeostasis2.Controllers.Feedforward.FeedforwardEx;
import homeostasis2.Filters.Estimators.Estimator;
import homeostasis2.Filters.Estimators.LowPassEstimator;
import homeostasis2.Parameters.FeedforwardCoefficientsEx;
import homeostasis2.Parameters.PIDCoefficients;
import homeostasis2.Parameters.PIDCoefficientsEx;
import homeostasis2.System.SISOsystem;

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

		SISOsystem sys = new SISOsystem(lowPassFilter,PID,feedforward);

		waitForStart();
		while (opModeIsActive()) {

		}
	}


}
