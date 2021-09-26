package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.WPILIB.TrapezoidProfile;
import org.firstinspires.ftc.teamcode.filter.positionKalmanFilter;
import org.firstinspires.ftc.teamcode.geometry.Vector2d;
import org.firstinspires.ftc.teamcode.geometry.Vector3D;
import org.firstinspires.ftc.teamcode.roadrunnerquickstart.SampleTankDrive;

import java.util.ArrayList;

import homeostasis.systems.DcMotorPlant;

import static org.firstinspires.ftc.teamcode.utils.utils.clipMotor;
import static org.firstinspires.ftc.teamcode.utils.utils.drawRobot;
import static org.firstinspires.ftc.teamcode.utils.utils.drawRobotBlue;
import static org.firstinspires.ftc.teamcode.utils.utils.drawRobotGreen;

public class drivetrain implements subsystem {



    /**
     * the maximum translational acceleration of our robot
     */
    public final static double MAX_ROBOT_ACCELERATION = 30;

    private long loopcount = 0;

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
    private boolean velocityNullPointerExceptionFlagTriggered = false;
    private SampleTankDrive positionEstimatorRR;

    private IMUOdom imuodom = new IMUOdom(this);
    private positionKalmanFilter positionEstimator;


    public Vector3D robotPosition = new Vector3D(0, 0, 0);
    private Vector3D previousRobotPosition = robotPosition;

    public Vector3D robotVelocity = new Vector3D(0, 0, 0);


    public DcMotorEx FrontLeft;
    public DcMotorEx FrontRight;
    public DcMotorEx BackLeft;
    public DcMotorEx BackRight;

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

        positionEstimatorRR = new SampleTankDrive(hwmap);

        ArrayList<DcMotorEx> leftMotors = new ArrayList<>();
        ArrayList<DcMotorEx> rightMotors = new ArrayList<>();
        leftMotors.add(FrontLeft);
        leftMotors.add(BackLeft);
        rightMotors.add(FrontRight);
        rightMotors.add(BackRight);

        leftMotorSys = new DcMotorPlant(leftMotors);
        rightMotorSys = new DcMotorPlant(rightMotors);

        imuodom.init(hwmap);
        positionEstimator = new positionKalmanFilter(robotPosition);

    }

    @Override
    public void initNoReset(HardwareMap hwmap) {
        init(hwmap);
    }

    @Override
    public void update() {
        updatePoseEstimate();
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

        fl = Range.clip(fl,-1,1);
        fr = Range.clip(fr, -1, 1);
        bl = Range.clip(bl, -1,1);
        br = Range.clip(br, -1, 1);


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

    public double getAngle() {
        return robotPosition.angle.getRadians();
    }



    /**
     * real gamer field relative driving
     * @param xSpeed the y speed of the robot
     * @param ySpeed the x speed of the robot
     * @param turnSpeed the turning speed of the robot
     */
    public void FieldRelative(double xSpeed, double ySpeed, double turnSpeed) {
        com.acmerobotics.roadrunner.geometry.Vector2d input = new com.acmerobotics.roadrunner.geometry.Vector2d(xSpeed,
                ySpeed).rotated(robotPosition.getAngleRadians());


        weightedRobotRelative(input.getX(), input.getY(), turnSpeed);
    }

    /**
     * old field relative driving
     *
     * @param xSpeed    the x speed of the robot
     * @param ySpeed    the y speed of the robot
     * @param turnSpeed the turning speed of the robot
     */
    public void legacyFieldRelative(double xSpeed, double ySpeed, double turnSpeed) {
        double angle = Math.toDegrees(robotPosition.getAngleRadians());
        xSpeed = clipMotor(xSpeed);
        ySpeed = clipMotor(ySpeed);
        turnSpeed = clipMotor(turnSpeed);


        Vector2d input = new Vector2d(xSpeed, ySpeed);
        input = input.rotateBy(angle);
        double theta = input.angle();

        double fl = input.magnitude() * Math.sin(theta + Math.PI / 4) + turnSpeed;
        double fr = input.magnitude() * Math.sin(theta - Math.PI / 4) - turnSpeed;
        double bl = input.magnitude() * Math.sin(theta - Math.PI / 4) + turnSpeed;
        double br = input.magnitude() * Math.sin(theta + Math.PI / 4) - turnSpeed;

        setMotorPowers(fl, fr, bl, br);

    }

    /**
     * legacy field relative with vector input
     *
     * @param input   vector3d for input powers
     * @param flipped flip y axis when true
     */
    public void legacyFieldRelative(Vector3D input, boolean flipped) {
        legacyFieldRelative(input.getX(), flipped ? -input.getY() : input.getY(), input.getAngleRadians());
    }


    /**
     * Field relative using Vector3D as input
     *
     * @param powers the 3 axis motor powers we should apply to the robot
     */
    public void FieldRelative(Vector3D powers) {
        FieldRelative(powers.getX(), powers.getY(), powers.getAngleRadians());
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

    /**
     * set robot relative motor powers, weighted
     * @param xSpeed forward and back relative to the robot
     * @param ySpeed side to side relative to the robot.
     *
     *               Because mecanum strafing has approximately 90% the efficiency in the
     *               strafing direction as the forward direction, we multiply by 100/90 to account for this.
     *
     * @param turnSpeed
     */
    public void weightedRobotRelative(double xSpeed, double ySpeed, double turnSpeed) {

        robotRelative(xSpeed,turnSpeed);
    }

    /**
     * field relative using Vector3D as input and with a reflected y axis power
     *
     * @param powers the three axis motor power we apply to the robot
     */
    public void FieldRelativeFlipped(Vector3D powers) {

        FieldRelative(powers.getX(), -powers.getY(), powers.getAngleRadians());

    }

    public void STOP() {
        setMotorPowers(0,0,0,0);
    }

    @Override
    public Vector3D subsystemState() {
        return robotPosition;
    }


    /**
     * set roadrunner localization to a desired estimate
     * @param estimate desired estimate that we want to
     */
    public void setPoseEstimate(Vector3D estimate) {

        robotPosition = estimate;
        positionEstimatorRR.setPoseEstimate(estimate.toPose2d());

    }

    /**
     * update odometry position estimation, should be called periodically throughout the opmode
     */
    public void updatePoseEstimate() {

        positionEstimatorRR.update();

        robotPosition = new Vector3D(positionEstimatorRR.getPoseEstimate());
        Vector3D positionDelta = robotPosition.getError(previousRobotPosition);
        positionDelta.setAngleRad(robotPosition.getAngleRadians());

        double xVelo;
        double yVelo;
        double thetaVelo;

        try {
            xVelo = positionEstimatorRR.getPoseVelocity().getX();
            yVelo = positionEstimatorRR.getPoseVelocity().getY();
            thetaVelo = positionEstimatorRR.getPoseVelocity().getHeading();

        } catch (NullPointerException e) {

            velocityNullPointerExceptionFlagTriggered = true;
            xVelo = 0;
            yVelo = 0;
            thetaVelo = 0;

        }

        Vector2d veloVector = new Vector2d(xVelo, yVelo);

        veloVector = veloVector.rotateBy(Math.toDegrees(positionEstimatorRR.getPoseEstimate().getHeading()));

        // if we have a packet, display our robots position on the dashboard


        robotVelocity = new Vector3D(veloVector.getX(), veloVector.getY(), thetaVelo);

        //robotPosition.setAngleRad(t265PoseEstimate.getAngleRadians());
        drawRobot(robotPosition, dashboard.packet);

//        Vector3D accel = positionEstimatorRR.getIMUAccel();
//        imuodom.updateWithAccel(accel);
//        drawRobot(imuodom.subsystemState(), dashboard.packet);
//        positionEstimator.updatePoseEstimate(positionDelta, imuodom.imuPosDelta);
//        System.out.println("kalman filtered pose" + positionEstimator.getPositionEstimate());
//        drawRobotGreen(imuodom.subsystemState(),dashboard.packet);
//        drawRobotBlue(positionEstimator.getPositionEstimate(),dashboard.packet);
//        dashboard.packet.put("x accel",accel.getX());
//        dashboard.packet.put("y accel",accel.getY());
//
//        dashboard.packet.put("x accel pos delta",imuodom.imuPosDelta.getX());
//        dashboard.packet.put("y accel pos delta",imuodom.imuPosDelta.getY());
//        dashboard.packet.put("x odom delta",positionDelta.getX());
//        dashboard.packet.put("y odom delta",positionDelta.getY());
//        dashboard.packet.put("kalman x delta",positionDelta.getX());
//        dashboard.packet.put("kalman y delta",positionDelta.getY());


        previousRobotPosition = robotPosition;



    }

    /**
     * get WPILIB constraints for the x,y movement of the robot
     *
     * @return the constraints
     */
    public TrapezoidProfile.Constraints getTranslationalConstraints() {
        return new TrapezoidProfile.Constraints(MAX_ROBOT_VELOCITY, MAX_ROBOT_ACCELERATION);
    }

    /**
     * get the WPILIB constraints for the theta movement of the robot
     *
     * @return the constraints
     */
    public TrapezoidProfile.Constraints getRotationalConstraints() {
        return new TrapezoidProfile.Constraints(MAX_ANGULAR_VELOCITY, MAX_ANGULAR_ACCELERATION);
    }

    /**
     * Inorder to ensure that the robot travels in a straight line and not an L shape towards to target position we need
     * to make sure that the robot will arrive at the target position for both x and y at the same time.
     * <p>
     * To accomplish this we can use our simple formula for displacement which states that:
     * <p>
     * position = velocity * time
     * <p>
     * Where we can get into trouble is that the shorter distance will be converged on faster than the longer position
     * <p>
     * so we set the velocity of the shorter distance to (short distance / long distance) * maximum velocity
     * This ensures that the robot arrives at the targetPose at approximately the same time as the long distance will be
     * traveling at the maximum velocity
     *
     * @param targetPose target position the robot needs to reach
     * @return an array containing the x, y constraints as calculated
     */
    public TrapezoidProfile.Constraints[] getOptimalConstraints(Vector3D targetPose) {

        Vector3D targetError = targetPose.getError(robotPosition);
        targetError = targetError.abs();

        TrapezoidProfile.Constraints xProfile;
        TrapezoidProfile.Constraints yProfile;

        double multiplier = Math.min(targetError.getX(), targetError.getY())
                / Math.max(targetError.getX(), targetError.getY());

        // if the x distance is longer then we need to slow down y
        if (targetError.getX() > targetError.getY()) {
            xProfile = new TrapezoidProfile.Constraints(MAX_ROBOT_VELOCITY, MAX_ROBOT_ACCELERATION);
            yProfile = new TrapezoidProfile.Constraints(MAX_ROBOT_VELOCITY * multiplier, MAX_ROBOT_ACCELERATION);
        } else {
            yProfile = new TrapezoidProfile.Constraints(MAX_ROBOT_VELOCITY, MAX_ROBOT_ACCELERATION);
            xProfile = new TrapezoidProfile.Constraints(MAX_ROBOT_VELOCITY * multiplier, MAX_ROBOT_ACCELERATION);
        }

        return new TrapezoidProfile.Constraints[]{xProfile, yProfile};


    }

    public Vector3D getRobotPosition() {
        return robotPosition;
    }

    public Vector3D getRobotVelocity() {
        return robotVelocity;
    }


    public void debugPosition(Telemetry telemetry) {
        telemetry.addData("x pos: ", robotPosition.getX());
        telemetry.addData("y pos: ", robotPosition.getY());
        telemetry.addData("theta pos", robotPosition.getAngleRadians());
        telemetry.addData("x velo", robotVelocity.getX());
        telemetry.addData("y velo: ", robotVelocity.getY());
        telemetry.addData("theta velo: ", robotVelocity.getAngleRadians());
    }

    /**
     * check if the velocity encountered a null pointer exception inorder to disable feedback
     *
     * @return if the velocity flag has been triggered.
     */
    public boolean isVelocityNullPointerExceptionFlagTriggered() {
        return velocityNullPointerExceptionFlagTriggered;
    }


}
