package org.firstinspires.ftc.teamcode.classicalControl;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.utils.utils.normalizedHeadingError;

public class NonlinearPID {

    // Proportional gain
    public double Kp = 0;
    // Integral gain
    public double Ki = 0;
    // derivative gain
    public double Kd = 0;
    // second order derivative gain
    public double Kd2 = 0;
    // feedforward gain
    public double Kf = 0;

    protected double integralSum = 0;

    protected double derivative = 0;

    protected double lastDerivative = 0;

    protected double derivativeSecondOrder = 0;

    protected double integralSumLimit = 1;

    protected double previousOutput = 0;

    // true when we want to low pass filter our derivative
    private boolean enableLowPassDerivative = false;
    // if we change the reference then we reset our integral sum
    private boolean integralResetOnSetpointChange = false;
    // limit integral sum
    private boolean limitIntegralSum = false;
    // wrap if we are dealing with angles
    private boolean wrapAngle = false;
    // reset integral sum if we cross over 0 error
    private boolean integralZeroCrossoverDetection = false;

    // low pass filter gain
    private double a = 0.8;
    private double a2 = 0.8;

    private double previousFilterEstimate = 0;
    private double previousFilterEstimate2 = 0;


    private double lastReference = 0;

    private double lastError = 0;

    private final ElapsedTime integralTimer = new ElapsedTime();


    public NonlinearPID(PIDFCoefficients pidf) {
        this.Kp = pidf.Kp;
        this.Ki = pidf.Ki;
        this.Kd = pidf.Kd;
        this.Kf = pidf.Kf;
        this.Kd2 = pidf.Kd2;
    }


    /**
     * calculate the output given an already computed error
     *
     * @param error the computed error
     * @return the pid output
     */
    public double calculateOutput(double error) {
        return calculateOutput(0, error);
    }


    /**
     * calculate the output based on the reference and the state
     *
     * @param reference where we would like to be
     * @param state     where we are
     * @return the resulting output
     */
    public double calculateOutput(double reference, double state) {

        // calculate error, wrap if we are dealing with angles
        double error;
        if (wrapAngle) {
            error = normalizedHeadingError(reference, state);
        } else {
            error = reference - state;
        }



        // forward euler integration to approximate the integral
        // only integrate if our output is less than the saturation limit
        double SATURATION_LIMIT = 1;
        if (Math.abs(previousOutput) <= SATURATION_LIMIT) {
            integralSum += integralTimer.seconds() * error;
        }

        // reset integral sum upon setpoint changes
        if (integralResetOnSetpointChange) {
            if (reference != lastReference) {
                integralSum = 0;
            }
        }
        lastReference = reference;

        // set a hard limit on the integral sum if applicable
        if (limitIntegralSum) {
            if (integralSum > integralSumLimit) {
                integralSum = integralSumLimit;
            } else if (integralSum < -integralSumLimit) {
                integralSum = -integralSumLimit;
            }
        }

        // calculate the derivative using a low pass filter
        if (enableLowPassDerivative) {
            derivative = lowPassFilter(error - lastError) / integralTimer.seconds();
            derivativeSecondOrder = lowPassFilter2((derivative - lastDerivative) / integralTimer.seconds());
        } else {
            derivative = (error - lastError) / integralTimer.seconds();
            derivativeSecondOrder = (derivative - lastDerivative) / integralTimer.seconds();
        }


        // if we cross over 0 error we want to reset the integral sum
        if (integralZeroCrossoverDetection) {
            // check for zero crossover
            if ((lastError > 0 && error < 0) || (lastError < 0 && error > 0)) {
                integralSum = 0;
            }
        }

        // compute parallel output sum
        double output = (error * Kp) + (integralSum * Ki) + (derivative * Kd) + (derivativeSecondOrder * Kd2) + (reference * Kf);

        lastError = error;
        lastDerivative = derivative;

        integralTimer.reset();
        previousOutput = output;

        return output;
    }

    /**
     * calculate Proportional and feedforward output only
     *
     * @param reference where we would like to be
     * @param state     where we are
     * @return the proportional plus feedforward output
     */
    public double calculateOutputPFOnly(double reference, double state) {
        return ((reference - state) * Kp) + (reference * Kf);
    }

    /**
     * generate estimate from low pass filter
     *
     * @param measurement the measurement
     * @return the estimate
     */
    public double lowPassFilter(double measurement) {
        double currentFilterEstimate = (a * previousFilterEstimate) + (1 - a) * measurement;
        previousFilterEstimate = currentFilterEstimate;
        return currentFilterEstimate;
    }

    /**
     * generate estimate from low pass filter
     *
     * @param measurement the measurement
     * @return the estimate
     */
    public double lowPassFilter2(double measurement) {
        double currentFilterEstimate = (a2 * previousFilterEstimate2) + (1 - a2) * measurement;
        previousFilterEstimate2 = currentFilterEstimate;
        return currentFilterEstimate;

    }


    public void setEnableLowPassDerivative(boolean enableLowPassDerivative) {
        this.enableLowPassDerivative = enableLowPassDerivative;
    }

    public void setA(double a) {
        this.a = a;
    }

    public void setIntegralResetOnSetpointChange(boolean integralResetOnSetpointChange) {
        this.integralResetOnSetpointChange = integralResetOnSetpointChange;
    }

    public void setLimitIntegralSum(boolean limitIntegralSum) {
        this.limitIntegralSum = limitIntegralSum;
    }

    public double getIntegralSumLimit() {
        return integralSumLimit;
    }

    public void setIntegralSumLimit(double integralSumLimit) {
        this.integralSumLimit = integralSumLimit;
    }

    public void setWrapAngle(boolean wrapAngle) {
        this.wrapAngle = wrapAngle;
    }

    public void setIntegralZeroCrossoverDetection(boolean integralZeroCrossoverDetection) {
        this.integralZeroCrossoverDetection = integralZeroCrossoverDetection;
    }
}
