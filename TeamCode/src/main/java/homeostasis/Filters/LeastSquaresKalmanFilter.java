package homeostasis.Filters;

import static org.firstinspires.ftc.teamcode.utils.utils.AngleWrap;

import android.os.Build;

import androidx.annotation.RequiresApi;

import org.firstinspires.ftc.teamcode.utils.LinearRegression;
import org.firstinspires.ftc.teamcode.utils.SizedStack;

public class LeastSquaresKalmanFilter {

    protected double Q;
    protected double R;
    protected int N;
    protected double P = 1;
    protected double K = 0;
    protected double x;
    protected boolean forAngles;
    protected SizedStack<Double> estimates;
    protected LinearRegression regression;

    /**
     * A kalman filter that uses a least squares regression as it's model.
     * @param Q Sensor Covariance
     * @param R Model Covariance
     * @param N Number of elements we can hold in our stack.
     * @param forAngles flag for if our measurements are angles.
     */
    public LeastSquaresKalmanFilter(double Q, double R, int N, boolean forAngles) {
        this.Q = Q;
        this.R = R;
        this.N = N;
        this.x = 0;
        this.forAngles = forAngles;
        this.estimates = new SizedStack<>(N);
        initializeStackWith0();
        regression = new LinearRegression(stackToDoubleArray());
        findK();
    }


    /**
     * public accessor for the update methods.  appropriately calls the correct one depending on
     * the type of measurement being used.
     * @param measurement the current measurement.
     * @return optimal state estimate.
     */
    @RequiresApi(api = Build.VERSION_CODES.N)
    public double update(double measurement) {
        if (forAngles) return updateAngle(measurement);
        return updateNormal(measurement);
    }


    /**
     * set the state estimate.
     * @param x state estimate
     */
    public void setX(double x) {
        this.x = x;
    }

    public double getX() {
        return x;
    }

    /**
     * update the kalman filter for traditional; continous values.
     * @param measurement the current measurement
     * @return the optimal state estimate.
     */
    @RequiresApi(api = Build.VERSION_CODES.N)
    protected double updateNormal(double measurement) {
        regression.runLeastSquares();
        x += regression.predictNextValue() - estimates.peek();
        x += K * (measurement - x);
        estimates.push(x);
        regression = new LinearRegression(stackToDoubleArray());
        return x;
    }

    /**
     * updates the kalman filter for angular values
     * @param measurement angle measurement
     * @return optimal estimate.
     */
    @RequiresApi(api = Build.VERSION_CODES.N)
    protected double updateAngle(double measurement) {
        regression.runLeastSquares();
        x = AngleWrap(x);
        x += regression.predictNextValue() - estimates.peek();
        x += K * AngleWrap(measurement - x);
        x = AngleWrap(x);
        estimates.push(x);
        regression = new LinearRegression(stackToDoubleArray());
        return x;
    }

    /**
     * Iteratively compute K using the D.A.R.E
     */
    public void findK() {
        for (int i = 0; i < 2000; ++i) solveDARE();
    }

    /**
     * solve the discrete time algebraic riccati equation (D.A.R.E)
     */
    public void solveDARE() {
        P = P + Q;
        K = P / (P + R);
        P = (1-K) * P;
    }

    /**
     * initialize the stack to all 0's
     */
    protected void initializeStackWith0() {
        for (int i = 0; i < N; ++i) {
            estimates.push(0.0);
        }
    }

    /**
     * convert the stack to an array of doubles
     * @return an array of doubles.
     */
    protected double[] stackToDoubleArray() {
        double[] newValues = new double[N];
        for (int i = 0; i < estimates.size(); ++i) {
            newValues[i] = estimates.get(i);
        }
        return  newValues;
    }

    public void setForAngles(boolean forAngles) {
        this.forAngles = forAngles;
    }
}
