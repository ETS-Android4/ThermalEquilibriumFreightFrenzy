package org.firstinspires.ftc.teamcode.commandBase.autoActions.Misc;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commandBase.action;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

public class MotorSineSweep implements action {

    public Robot robot;
    public ElapsedTime timer;
    public double gain = 4;
    public MotorSineSweep(Robot robot) {
        this.robot = robot;
        this.timer = new ElapsedTime();
    }

    @Override
    public void startAction() {

    }

    @Override
    public void runAction() {
        double power = Math.sin(timer.seconds() / gain);
        robot.driveTrain.robotRelative(power,0);
    }

    @Override
    public void stopAction() {

    }

    @Override
    public boolean isActionComplete() {
        return false;
    }

    @Override
    public boolean isActionPersistent() {
        return false;
    }
}
