package homeostasis.Filters;

import static org.firstinspires.ftc.teamcode.utils.utils.AngleWrap;

import android.os.Build;

import androidx.annotation.RequiresApi;

import org.firstinspires.ftc.teamcode.utils.LinearRegression;
import org.firstinspires.ftc.teamcode.utils.SizedStack;

public class LeastSquaresKalmanFilter {

    public double Q;
    public double R;
    public int N;
    public double P = 1;
    public double K = 0;
    public double x;
    public boolean forAngles;
    protected SizedStack<Double> estimates;
    protected LinearRegression regression;

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



    @RequiresApi(api = Build.VERSION_CODES.N)
    public double update(double measurement) {
        if (forAngles) return updateAngle(measurement);
        return updateNormal(measurement);
    }


    public void setX(double x) {
        this.x = x;
    }

    @RequiresApi(api = Build.VERSION_CODES.N)
    protected double updateNormal(double measurement) {


        regression.runLeastSquares();
       // System.out.println("x before advance is " + x + " next prediction is " + regression.predictNextValue() + " previous estimate is " + estimates.peek());
        x += regression.predictNextValue() - estimates.peek();
        //System.out.println("x after advance is " + x);
        x += K * (measurement - x);

        estimates.push(measurement);
        regression = new LinearRegression(stackToDoubleArray());
        return x;
    }

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

    protected void initializeStackWith0() {
        for (int i = 0; i < N; ++i) {
            estimates.push(0.0);
        }
    }

    protected double[] stackToDoubleArray() {
        double[] newValues = new double[N];
        for (int i = 0; i < estimates.size(); ++i) {
            newValues[i] = estimates.get(i);
        }
        return  newValues;
    }

}
