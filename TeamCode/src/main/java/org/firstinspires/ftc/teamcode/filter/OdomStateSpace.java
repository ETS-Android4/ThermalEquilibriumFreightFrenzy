package org.firstinspires.ftc.teamcode.filter;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.geometry.Vector2d;
import org.firstinspires.ftc.teamcode.geometry.Vector3D;
import org.firstinspires.ftc.teamcode.roadrunnerquickstart.util.Encoder;

import static org.firstinspires.ftc.teamcode.utils.utils.normalizeAngleRR;

/**
 * state space method for obtaining odometry estimates using the pose exponential technique.
 */
public class OdomStateSpace {

    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = 0.74803;//0.86;//0.74803; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double LATERAL_DISTANCE = 13.540597428955047; // in; distance between the left and right wheels
    public static double FORWARD_OFFSET = -1.73; // in; offset of the lateral wheel

    private Encoder leftEncoder;
    private Encoder rightEncoder;
    private Encoder frontEncoder;

    private double lastLeftEncoderPos = 0;
    private double lastRightEncoderPos = 0;
    private double lastFrontEncoderPos = 0;

    private double xG = 0;
    private double yG = 0;
    private double thetaG = 0;
    private Vector3D poseEstimate;

    /**
     * construct odometry within initial conditions and encoder hardware map
     *
     * @param initialPosition
     * @param hwmap
     */
    public OdomStateSpace(Vector3D initialPosition, HardwareMap hwmap) {
        poseEstimate = initialPosition;
        this.xG = initialPosition.getX();
        this.yG = initialPosition.getY();
        this.thetaG = initialPosition.getAngleRadians();
        setEncoder(hwmap);
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    /**
     * update the odom estimate given the current encoder position
     *
     * @return current pose estimate
     */
    public Vector3D updateRawOdometryEstimate() {

        // get encoder positions
        double left = encoderTicksToInches(leftEncoder.getCurrentPosition());
        double right = encoderTicksToInches(rightEncoder.getCurrentPosition());
        double front = encoderTicksToInches(frontEncoder.getCurrentPosition());
        // calculate odometry deltas
        double deltaL = left - lastLeftEncoderPos;
        double deltaR = right - lastRightEncoderPos;
        double deltaF = front - lastFrontEncoderPos;
        // save current position for next delta
        lastLeftEncoderPos = left;
        lastRightEncoderPos = right;
        lastFrontEncoderPos = front;
        // calculate robot relative movements
        double xRelative = (WHEEL_RADIUS / 2) * (deltaL + deltaR);
        double yRelative = WHEEL_RADIUS * ((FORWARD_OFFSET / LATERAL_DISTANCE) * (deltaL - deltaR) + deltaF);
        System.out.println("Yrelative is + " + yRelative);
        double thetaRelative = (WHEEL_RADIUS / LATERAL_DISTANCE) * (deltaR - deltaL);

        // pose exponentials!
        double currentTheta = thetaG;

        // sin of theta relative
        double sThetaR = Math.sin(thetaRelative);
        // cos of current theta
        double cTheta = Math.cos(currentTheta);
        // sin of current theta
        double sTheta = Math.sin(currentTheta);
        // negative cos theta relative + 1
        double cosThetaRelative = -Math.cos(thetaRelative) + 1;
        double x = xRelative;
        double y = yRelative;

        /*
        if (thetaRelative == 0) {
            thetaRelative = EPSILON;
        }

            // calculate global x position
            x = xRelative * ((sThetaR * cTheta) - sTheta * cosThetaRelative);
            x = x + (yRelative * (-sThetaR * sTheta - cTheta * cosThetaRelative));
            x = x / thetaRelative;

            // calculate global y position
            y = xRelative * (sThetaR * sTheta + cTheta * cosThetaRelative);

            System.out.println("y is now " + y );
            y += yRelative * (sThetaR * cTheta - sTheta * cosThetaRelative);
            System.out.println("trig thing : " + (sThetaR * cTheta - sTheta * cosThetaRelative));
            System.out.println("y is now now " + y);
            y /= thetaRelative;
            System.out.println("y is now now " + y);

         */

        double sineterm = 0;
        double costerm = 0;

        if (EpsilonEquals(thetaRelative, 0)) {
            sineterm = 1.0 - thetaRelative * thetaRelative / 6.0;
            costerm = thetaRelative / 2;
        } else {
            sineterm = Math.sin(thetaRelative) / thetaRelative;
            costerm = (1 - Math.cos(thetaRelative)) / thetaRelative;
        }

        x = sineterm * xRelative - costerm * yRelative;
        y = costerm * xRelative + sineterm * yRelative;

        Vector2d output = new Vector2d(x, y).rotateBy(thetaG);

        thetaG += thetaRelative;
        thetaG = normalizeAngleRR(thetaG);

        xG += output.getX();
        yG += output.getY();
        poseEstimate = new Vector3D(xG, yG, thetaG);

        return poseEstimate;


    }

    /**
     * check if two values are almost equal
     *
     * @param val1 value 1
     * @param val2 value 2
     * @return if the items are less than 1e-6 apart
     */
    private boolean EpsilonEquals(double val1, double val2) {

        return (val1 - val2) < 1e-6;

    }

    /**
     * @return get the position estimate from odom
     */
    public Vector3D getPoseEstimate() {
        return poseEstimate;
    }

    /**
     * Set the position of the odometry estimate
     *
     * @param poseEstimate the position estimate
     */
    public void setPoseEstimate(Vector3D poseEstimate) {
        this.poseEstimate = poseEstimate;
    }

    /**
     * initialize hardware map for the encoder
     *
     * @param hwmap
     */
    private void setEncoder(HardwareMap hwmap) {
        leftEncoder = new Encoder(hwmap.get(DcMotorEx.class, "FrontLeft"));
        rightEncoder = new Encoder(hwmap.get(DcMotorEx.class, "FrontRight"));
        frontEncoder = new Encoder(hwmap.get(DcMotorEx.class, "BackLeft"));

        leftEncoder.setDirection(Encoder.Direction.REVERSE);
    }

    /**
     * get the previous left encoder position
     *
     * @return the previous left encoder position
     */
    public double getLastLeftEncoderPos() {
        return lastLeftEncoderPos;
    }

    /**
     * get the previous right encoder position
     *
     * @return the previous right encoder position
     */
    public double getLastRightEncoderPos() {
        return lastRightEncoderPos;
    }

    /**
     * get the previous front encoder position
     *
     * @return the previous front encoder position
     */
    public double getLastFrontEncoderPos() {
        return lastFrontEncoderPos;
    }
}
