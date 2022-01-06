package org.firstinspires.ftc.teamcode.subsystems.scoringMechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.subsystem;

import homeostasis.utils.State;


/**
 * main class for the entire deposit including bucket, slides, and virtual 4 bar
 */
public class Deposit implements subsystem {

	public double slideErrorTolerance = 5;
	protected Slides slideSystem = new Slides();
	protected Virtual4bar v4b = new Virtual4bar();
	protected depositStates state = depositStates.IN;

	@Override
	public void init(HardwareMap hwmap) {
		slideSystem.init(hwmap);
		v4b.init(hwmap);
	}

	@Override
	public void initNoReset(HardwareMap hwmap) {
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
	public State subsystemState() {
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

	public boolean tolerantEnoughForDeploy() {
		return Math.abs(slideSystem.getControllerError()) < 10;
	}

	public enum depositStates {
		DISARMED, // motor power is cut
		IN, // everything is in, ready for going over
		COLLECTION,

		GOING_TO_HIGH,
		GOING_TO_MID,
		GOING_TO_LOW,
		AT_HIGH, // at high but not deposited
		AT_MID, // at mid but not deposited
		AT_LOW, // at low but not deposited
		AT_CAPPING,
		DEPOSITING, //depositing
		GOING_IN
	}
}

