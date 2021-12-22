package org.firstinspires.ftc.teamcode.controls;

import com.qualcomm.robotcore.util.ElapsedTime;

public class BangBangController {

    protected BangBangParameters controllerParams;

    protected double previousError = 0;

    ElapsedTime timer = new ElapsedTime();

    public BangBangController(BangBangParameters controllerParams) {
        this.controllerParams = controllerParams;
    }

    public double controlOutput(double error) {
        if (controllerParams.hysteresis > Math.abs(error)) return 0;
        return Math.signum(error) * controllerParams.maxOutput
                + getDerivative(error) * controllerParams.kd;
    }


    public double getDerivative(double error) {
        double derivative = derivativeCalculation(error, previousError, timer.seconds());
        previousError = error;
        timer.reset();
        return derivative;
    }

    public double derivativeCalculation(double error, double previousError, double dt) {
        return (error - previousError) / dt;
    }

}
