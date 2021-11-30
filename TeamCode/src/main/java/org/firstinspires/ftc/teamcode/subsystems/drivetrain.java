package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.classicalControl.velocityControl;

import java.util.ArrayList;

import homeostasis.systems.DcMotorPlant;

import static org.firstinspires.ftc.teamcode.subsystems.robot.isCompBot;


public class drivetrain implements subsystem {


    public static final double MAX_DRIVE_MOTOR_TPS = 2 * (435 * (15.0 / 12) * 60.0) / 28;

    /**
     * the maximum translational acceleration of our robot
     */
    public final static double MAX_ROBOT_ACCELERATION = 30;

    public static final double ROBOT_SIZE = 17;
    public static final double ROBOT_RADIUS = ROBOT_SIZE / 2;
    // inches per second
    /**
     * the maximum velocity of our robot
     */
    public final static double MAX_ROBOT_VELOCITY = 30;//55;

    /**
     * the maximum angular speed of our robot
     */
    public final static double MAX_ANGULAR_VELOCITY = Math.toRadians(300);
    /**
     * the maximum angular acceleration of our robot
     */
    public final static double MAX_ANGULAR_ACCELERATION = Math.toRadians(240);

    public DcMotorEx FrontLeft;
    public DcMotorEx FrontRight;
    public DcMotorEx BackLeft;
    public DcMotorEx BackRight;

    public double last_FrontLeft_Power = 0;
    public double last_BackRight_Power = 0;

    public DcMotorPlant leftMotorSys;
    public DcMotorPlant rightMotorSys;

    protected velocityControl leftMotorController;
    protected velocityControl rightMotorController;

    protected VoltageSensor batterySensor;

    protected HardwareMap hwmap;

    public drivetrain(VoltageSensor batterySensor) {
        this.batterySensor = batterySensor;
    }

    @Override
    public void init(HardwareMap hwmap) {
        this.hwmap = hwmap;


        FrontLeft = hwmap.get(DcMotorEx.class, "FrontLeft");
        FrontRight = hwmap.get(DcMotorEx.class, "FrontRight");
        BackLeft = hwmap.get(DcMotorEx.class, "BackLeft");
        BackRight = hwmap.get(DcMotorEx.class, "BackRight");

        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FrontLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        FrontRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        BackLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        BackRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (isCompBot) {
            FrontLeft.setDirection(DcMotorEx.Direction.REVERSE);
            FrontRight.setDirection(DcMotorEx.Direction.FORWARD);
            BackLeft.setDirection(DcMotorEx.Direction.REVERSE);
            BackRight.setDirection(DcMotorEx.Direction.FORWARD);

        } else {
            FrontLeft.setDirection(DcMotorEx.Direction.FORWARD);
            FrontRight.setDirection(DcMotorEx.Direction.REVERSE);
            BackLeft.setDirection(DcMotorEx.Direction.FORWARD);
            BackRight.setDirection(DcMotorEx.Direction.REVERSE);
        }


        ArrayList<DcMotorEx> leftMotors = new ArrayList<>();
        ArrayList<DcMotorEx> rightMotors = new ArrayList<>();
        leftMotors.add(FrontLeft);
        leftMotors.add(BackLeft);
        rightMotors.add(FrontRight);
        rightMotors.add(BackRight);

        leftMotorSys = new DcMotorPlant(leftMotors);
        rightMotorSys = new DcMotorPlant(rightMotors);

        leftMotorController = new velocityControl(leftMotorSys);
        rightMotorController = new velocityControl(rightMotorSys);

    }

    @Override
    public void initNoReset(HardwareMap hwmap) {
        init(hwmap);
    }

    @Override
    public void update() {

    }

    /**
     * sets the motor powers using lynx optimizations
     *
     * We don't actually update the power unless it is different than how the drive train is already moving
     * This is because we are sending unnecessary lynx writes to the rev hub, increasing loop times
     *
     *
     * @param left front left power (-1 < x < 1)
     * @param right front right power (-1 < x < 1)
     */
    public void setMotorPowers(double left, double right) {

        left = Range.clip(left,-1,1) * MAX_DRIVE_MOTOR_TPS;
        right = Range.clip(right,-1,1) * MAX_DRIVE_MOTOR_TPS;
        batterySensor = hwmap.voltageSensor.iterator().next();
        double voltage = batterySensor.getVoltage();
        leftMotorController.voltageCorrectedControl(left,voltage);
        rightMotorController.voltageCorrectedControl(right,voltage);

    }

    public void setMotorPowersRaw(double left, double right) {
        left = Range.clip(left,-1,1);
        right = Range.clip(right,-1,1);
        leftMotorSys.input(left);
        rightMotorSys.input(right);
    }

    /**
     * set robot relative motor powers
     *
     * @param xSpeed    speed in robot forward direction
     * @param turnSpeed turning speed
     */
    public void robotRelative(double xSpeed, double turnSpeed) {
        double leftPower = xSpeed + turnSpeed;
        double rightPower = xSpeed - turnSpeed;
        setMotorPowers(leftPower, rightPower);
    }

    public void robotRelativeRaw(double xSpeed, double turnSpeed) {
        double leftPower = xSpeed + turnSpeed;
        double rightPower = xSpeed - turnSpeed;
        setMotorPowersRaw(leftPower, rightPower);
    }

    public void robotRelativeRawClipped(double xSpeed, double turnSpeed, double max) {
        xSpeed = Range.clip(xSpeed,-max,max);
        turnSpeed = Range.clip(turnSpeed,-max,max);
        double leftPower = xSpeed + turnSpeed;
        double rightPower = xSpeed - turnSpeed;
        setMotorPowersRaw(leftPower, rightPower);
    }
    public void STOP() {
        setMotorPowers(0,0);
    }

    @Override
    public Object subsystemState() {
        return null;
    }


}
