package org.firstinspires.ftc.teamcode.controls;

public class BangBangParameters {
    public double maxOutput;
    public double hysteresis;
    public double kd;
    public double ki;
    public BangBangParameters(double maxOutput, double hysteresis, double Kd, double Ki) {
        this.maxOutput = maxOutput;
        this.hysteresis = hysteresis;
        this.kd = Kd;
        this.ki = Ki;
    }
}
