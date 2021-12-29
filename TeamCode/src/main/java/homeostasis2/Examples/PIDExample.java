package homeostasis2.Examples;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.function.DoubleSupplier;

import homeostasis2.Controllers.BasedPID;
import homeostasis2.Controllers.BasicPID;
import homeostasis2.Filters.Estimators.Estimator;
import homeostasis2.Filters.Estimators.LowPassEstimator;
import homeostasis2.Filters.Estimators.NoEstimator;
import homeostasis2.Parameters.PIDCoefficients;
import homeostasis2.SISOsystem;

public class PIDExample extends LinearOpMode {
	PIDCoefficients coefficients = new PIDCoefficients(1,0,0);
	protected DcMotorEx motor;

	@RequiresApi(api = Build.VERSION_CODES.N)
	@Override
	public void runOpMode() throws InterruptedException {


		waitForStart();
		while (opModeIsActive()) {
		}
	}


}
