package org.firstinspires.ftc.teamcode.controls;

import static org.firstinspires.ftc.teamcode.controls.controllerCoefficients.protoBotAntiTip;

import android.os.Build;

import androidx.annotation.RequiresApi;

import org.firstinspires.ftc.teamcode.utils.utils;

import java.util.function.DoubleSupplier;

public class AntiTipController {
    DoubleSupplier robotAngle;
    BangBangController controller = new BangBangController(protoBotAntiTip);

    protected double reference = Math.toRadians(-90);

    public AntiTipController(DoubleSupplier robotAngle) {
        this.robotAngle = robotAngle;
    }

    @RequiresApi(api = Build.VERSION_CODES.N)
    public double antiTip() {
        System.out.println("current antitip angle is " + robotAngle.getAsDouble());
        return controller.controlOutput(utils.normalizedHeadingError(reference,
                                                                     robotAngle.getAsDouble()));
    }

}
