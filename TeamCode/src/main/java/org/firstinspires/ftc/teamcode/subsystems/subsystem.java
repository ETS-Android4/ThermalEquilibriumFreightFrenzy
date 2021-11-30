package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

public interface subsystem {

    /**
     * nominal initialization, initialize and reset the state of the subsystem
     * @param hwmap hardware map instance
     */
    void init(HardwareMap hwmap);

    /**
     * initialize while keeping the state of the subsystem
     *
     * in the event that your system has no critical state,
     * make this method call {@link #init(HardwareMap)}
     *
     * @param hwmap hardware map instance
     */
    void initNoReset(HardwareMap hwmap);

    /**
     * called periodically to update the subsystems state
     */
    void update();

    /**
     * Expose the state of the subsystem
     *
     * @return an object that defines the state of your system.
     */
    Object subsystemState();

}
