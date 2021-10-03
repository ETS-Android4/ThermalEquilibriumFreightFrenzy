package org.firstinspires.ftc.teamcode.classicalControl;


import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HenoGoat.EasyOnlineMotionProfile;
import org.firstinspires.ftc.teamcode.WPILIB.TrapezoidProfile;
import org.firstinspires.ftc.teamcode.subsystems.dashboard;

public class ProfiledPIDController extends NonlinearPID {

    protected double MAX_ACCELERATION;
    protected double MAX_VELOCITY;
    protected TrapezoidProfile motionProfile;
    protected EasyOnlineMotionProfile profile;
    protected ElapsedTime profileTimer;
    protected TrapezoidProfile.Constraints systemConstraints;

    boolean hasRun = false;

    double scaler = 0;
    double rateOfChange = 0.025;

    double lastReference = 999999;

    /**
     * construct the PID controller and provide maximum velocity and acceleration constraints
     *
     * @param pidf             coefficients
     * @param MAX_VELOCITY     maximum constrained velocity
     * @param MAX_ACCELERATION maximum constrained acceleration
     */
    public ProfiledPIDController(PIDFCoeffecients pidf, double MAX_VELOCITY, double MAX_ACCELERATION) {
        super(pidf);
        this.MAX_ACCELERATION = MAX_ACCELERATION;
        this.MAX_VELOCITY = MAX_VELOCITY;
        this.systemConstraints = new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCELERATION);
    }
    public ProfiledPIDController(PIDFCoeffecients pidf) {
        super(pidf);
        this.MAX_ACCELERATION = 0;
        this.MAX_VELOCITY = 0;
        this.systemConstraints = new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCELERATION);
    }

    /**
     * given a reference and a state, compute the motion profile and then follow it
     *
     *
     * THIS METHOD DOES NOT OVERRIDE.
     *
     * @param reference the desired system state
     * @param state     the current system state
     * @return output 'u' to the plant
     */
    public double calculateProfiledOutput(double reference, double state) {

        if (reference != lastReference) {
            hasRun = false;
        }

        if (!hasRun) {
            profileTimer = new ElapsedTime();
            profileTimer.reset();
//            motionProfile = new TrapezoidProfile(systemConstraints, new TrapezoidProfile.State(reference, 0),
//                    new TrapezoidProfile.State(state, 0));
            profile = new EasyOnlineMotionProfile(MAX_VELOCITY, MAX_ACCELERATION);
            scaler = 0;
            hasRun = true;
        }

        if (scaler > 1) {
            scaler =  1;
        }
        scaler += rateOfChange;


        profile.updateProfile(reference - state);
        dashboard.packet.put("motion profiled reference is ",profile.getPosition());
        lastReference = reference;

        return calculateOutput(profile.getPosition(),state) * scaler;

    }


}
