package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import java.util.ArrayList;

import homeostasis.systems.DcMotorPlant;


public class drivetrain implements subsystem {



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

    public double fl = 0;
    public double fr = 0;
    public double bl = 0;
    public double br = 0;


    public double last_FrontLeft_Power = 0;
    public double last_FrontRight_Power = 0;
    public double last_BackRight_Power = 0;
    public double last_BackLeft_Power = 0;

    public DcMotorPlant leftMotorSys;
    public DcMotorPlant rightMotorSys;

    @Override
    public void init(HardwareMap hwmap) {
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

        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        FrontLeft.setDirection(DcMotorEx.Direction.FORWARD);
        FrontRight.setDirection(DcMotorEx.Direction.REVERSE);
        BackLeft.setDirection(DcMotorEx.Direction.FORWARD);
        BackRight.setDirection(DcMotorEx.Direction.REVERSE);


        ArrayList<DcMotorEx> leftMotors = new ArrayList<>();
        ArrayList<DcMotorEx> rightMotors = new ArrayList<>();
        leftMotors.add(FrontLeft);
        leftMotors.add(BackLeft);
        rightMotors.add(FrontRight);
        rightMotors.add(BackRight);

        leftMotorSys = new DcMotorPlant(leftMotors);
        rightMotorSys = new DcMotorPlant(rightMotors);

    }

    @Override
    public void initNoReset(HardwareMap hwmap) {
        init(hwmap);
    }

    @Override
    public void update() {
        //updatePoseEstimate();
    }

    /**
     * sets the motor powers using lynx optimizations
     *
     * We don't actually update the power unless it is different than how the drive train is already moving
     * This is because we are sending unnecessary lynx writes to the rev hub, increasing loop times
     *
     *
     * @param fl front left power
     * @param fr front right power
     * @param bl back left power
     * @param br back right power
     */
    public void setMotorPowers(double fl, double fr, double bl, double br) {

        this.fl = Range.clip(fl,-1,1);
        this.fr = Range.clip(fr, -1, 1);
        this.bl = Range.clip(bl, -1,1);
        this.br = Range.clip(br, -1, 1);


        if (last_FrontLeft_Power != fl) {
            FrontLeft.setPower(fl);
        }
        if (last_FrontRight_Power != fr) {
            FrontRight.setPower(fr);
        }
        if (last_BackLeft_Power != bl) {
            BackLeft.setPower(bl);
        }

        if (last_BackRight_Power != br) {
            BackRight.setPower(br);
        }

        last_BackLeft_Power = bl;
        last_BackRight_Power = br;
        last_FrontLeft_Power = fl;
        last_FrontRight_Power = fr;

    }

    /**
     * set robot relative motor powers
     *
     * @param xSpeed    speed in robot forward direction
     * @param turnSpeed turning speed
     */
    public void robotRelative(double xSpeed, double turnSpeed) {
        double frontLeftPower = xSpeed + turnSpeed;
        double backLeftPower = xSpeed + turnSpeed;
        double frontRightPower = xSpeed - turnSpeed;
        double backRightPower = xSpeed - turnSpeed;
        setMotorPowers(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
    }


    public void STOP() {
        setMotorPowers(0,0,0,0);
    }

    @Override
    public Object subsystemState() {
        return null;
    }


}
