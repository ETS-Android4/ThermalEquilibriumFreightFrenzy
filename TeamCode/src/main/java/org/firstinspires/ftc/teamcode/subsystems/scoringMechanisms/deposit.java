package org.firstinspires.ftc.teamcode.subsystems.scoringMechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.subsystem;


/**
 * main class for the entire deposit including bucket, slides, and virtual 4 bar
 */
public class deposit implements subsystem {

	public double slideErrorTolerance = 5;
	protected slides slideSystem;
	protected virtual4Bar v4b;
	protected bucket bucketSystem;
	protected depositStates state = depositStates.IN;

	@Override
	public void init(HardwareMap hwmap) {
		slideSystem = new slides();
		v4b = new virtual4Bar();
		bucketSystem = new bucket();

		bucketSystem.init(hwmap);
		slideSystem.init(hwmap);
		v4b.init(hwmap);
	}

	@Override
	public void initNoReset(HardwareMap hwmap) {
		slideSystem = new slides();
		v4b = new virtual4Bar();
		bucketSystem = new bucket();

		bucketSystem.initNoReset(hwmap);
		slideSystem.initNoReset(hwmap);
		v4b.initNoReset(hwmap);
	}

	@Override
	public void update() {

		slideSystem.setState(state);
		v4b.setState(state);

		slideSystem.update();
		v4b.update();

	}

	public depositStates getState() {
		return state;
	}

	public void setState(depositStates state) {
		this.state = state;
	}

	/**
	 * for now just return the position of the slides
	 *
	 * @return position of the slides
	 */
	@Override
	public homeostasis.utils.state subsystemState() {
		return slideSystem.subsystemState();
	}

	/**
	 * assess if our linear slides are close enough to the reference to continue
	 *
	 * @return true if we are within tolerance to the reference
	 */
	public boolean isSlideWithinTolerance() {
		return Math.abs(slideSystem.getControllerError()) < slideErrorTolerance;
	}

	public enum depositStates {
		DISARMED, // motor power is cut
		IN, // everything is in, ready for going over
		COLLECTION,
		AT_HIGH, // at high but not deposited
		AT_MID, // at mid but not deposited
		AT_LOW, // at low but not deposited
		DEPOSITING //depositing
	}
}

