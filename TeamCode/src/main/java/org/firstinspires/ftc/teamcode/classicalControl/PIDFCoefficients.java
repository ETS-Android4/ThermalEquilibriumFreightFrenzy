package org.firstinspires.ftc.teamcode.classicalControl;

public class PIDFCoefficients {

    public double Kp; // proportional gain
    public double Ki; // integral gain
    public double Kd; // derivative gain
    public double Kd2; // second order derivative
    public double Kf; // feedforward (velocity control only)
    public double H; // minimum output (hysteresis)

    public PIDFCoefficients(double Kp, double Ki, double Kd, double Kd2, double Kf) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.Kf = Kf;
        this.Kd2 = Kd2;
    }

    public PIDFCoefficients(double Kp, double Ki, double Kd, double Kd2, double Kf, double H) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.Kf = Kf;
        this.Kd2 = Kd2;
        this.H = H;
    }

    public PIDFCoefficients(double Kp, double Ki, double Kd, double Kf) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.Kd2 = 0;
        this.Kf = Kf;
        this.H = 0;
    }

    public PIDFCoefficients(double Kp, double Ki, double Kd) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.Kd2 = 0;
        this.Kf = 0;
        this.H = 0;
    }


    /**
     * create a duplicate object without the feedforward terms
     * @return feedback only coefficients
     */
    public PIDFCoefficients noFeedforward() {
        return new PIDFCoefficients(this.Kp,this.Ki,this.Kd,0,0,0);
    }


    /**
     * After increasing Kp to where the oscillations stop growing but do not decay, this is our critical gain.
     * Use this function to generate decent PID gains from this 'critical gain'
     * @param criticalGain Kp at which the system remains in a stable oscillation
     * @return tuned coefficients.
     */
    public static PIDFCoefficients JaRule(double criticalGain) {
        return new PIDFCoefficients(criticalGain / 1.3, criticalGain / 5, criticalGain / 7.7, 0, 0, criticalGain/24);
    }

}
