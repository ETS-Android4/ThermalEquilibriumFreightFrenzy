package org.firstinspires.ftc.teamcode.commandBase.teleopActions;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.apache.commons.math3.geometry.spherical.oned.Arc;
import org.firstinspires.ftc.teamcode.controls.AntiTipController;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

import java.util.function.DoubleSupplier;

public class SafeArcadeDrive extends ArcadeDrive {
    public AntiTipController controller;

    @RequiresApi(api = Build.VERSION_CODES.N)
    public SafeArcadeDrive(Robot robot, Gamepad gamepad1, Gamepad gamepad2) {
        super(robot, gamepad1, gamepad2);
        this.controller = new AntiTipController(new DoubleSupplier() {
            @Override
            public double getAsDouble() {
                return robot.odometry.getPitchAngle();
            }
        });
    }

    @RequiresApi(api = Build.VERSION_CODES.N)
    @Override
    public void periodic() {
        robot.driveTrain.robotRelativeRaw(-gamepad1.right_stick_y + controller.antiTip(),
                gamepad1.left_stick_x);
    }
}
