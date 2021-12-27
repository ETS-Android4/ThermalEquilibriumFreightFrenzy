package org.firstinspires.ftc.teamcode.commandBase.teleopActions;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.apache.commons.math3.geometry.spherical.oned.Arc;
import org.firstinspires.ftc.teamcode.controls.AntiTipController;
import org.firstinspires.ftc.teamcode.subsystems.Dashboard;
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
        double antiTipTipPower = controller.antiTip();
        Dashboard.packet.put("ANTI TIP output",antiTipTipPower * 50);
        robot.driveTrain.robotRelativeRaw(-gamepad1.right_stick_y + antiTipTipPower,
                gamepad1.left_stick_x);
    }
}
