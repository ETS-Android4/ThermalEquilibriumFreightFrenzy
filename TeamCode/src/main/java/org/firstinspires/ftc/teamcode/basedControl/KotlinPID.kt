package org.firstinspires.ftc.teamcode.basedControl

import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.classicalControl.PIDFCoefficients

class KotlinPID(var coefficients: PIDFCoefficients, var reference: Double) {
    var timer = ElapsedTime()
    var integral = 0.0
    var derivative = 0.0
    var lastError = 0.0
    var error = 0.0

    init {
        timer.reset();
    }

    fun update(reference: Double,state: Double) : Double {
        error = computeError(reference,state)
        val dt = timer.seconds()
        computeIntegral(dt)
        computeDerivative(dt)
        timer.reset()
        return output()
    }

    private fun computeError(reference: Double, state: Double) : Double = reference - state

    private fun computeIntegral(dt: Double) {
        integral += error * dt;
    }

    private fun computeDerivative(dt: Double) {
        derivative = (error - lastError) / dt
    }

    private fun output() = error * coefficients.Kp + integral * coefficients.Ki + derivative * coefficients.Kd
}

