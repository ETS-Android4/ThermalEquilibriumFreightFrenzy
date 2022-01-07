package org.firstinspires.ftc.teamcode.subsystems.scoringMechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.controls.RobustPID;
import org.firstinspires.ftc.teamcode.controls.controllerCoefficients;
import org.firstinspires.ftc.teamcode.subsystems.subsystem;

import java.util.ArrayList;

import homeostasis.systems.DcMotorPlant;
import homeostasis.utils.State;

public class Slides implements subsystem {

	protected RobustPID slideController = new RobustPID(controllerCoefficients.slideCoefficients, 0, 5, 0.1, 2);
	protected DcMotorPlant slides;
	protected DcMotorEx left;
	protected DcMotorEx right;

	protected Deposit.depositStates state = Deposit.depositStates.IN;

	protected double IN = 0;
	protected double COLLECTION = 0;
	protected double LOW = 765;
	protected double MID = 765;
	protected double HIGH = 765; // tune this imo

	protected double MAX_POWER = 1;
	protected double MAX_POWER_AT_CAP = 0.5;
	protected double CURRENT_MAX_POWER = 1;


	protected double FOR_CAPPING = 745;
	protected double FOR_CAPPING_TWO = 300;

	protected double referencePosition = 0;

	protected double error = 100;

	@Override
	public void init(HardwareMap hwmap) {
		initNoReset(hwmap);
		slides.resetEncoder();
	}

	@Override
	public void initNoReset(HardwareMap hwmap) {
		left = hwmap.get(DcMotorEx.class, "slideLeft");
		right = hwmap.get(DcMotorEx.class, "slideRight");

		right.setDirection(DcMotorSimple.Direction.REVERSE);
		left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

		ArrayList<DcMotorEx> motors = new ArrayList<>();
		motors.add(right);
		motors.add(left);
		slides = new DcMotorPlant(motors);
	}

	/**
	 * periodically called, updates state machine
	 */
	@Override
	public void update() {
		// if we are disarmed, we need not send power to our slides.
		if (state.equals(Deposit.depositStates.DISARMED)) {
			slides.input(0);
			return;
		}

		switch (state) {
			case IN:
				CURRENT_MAX_POWER = MAX_POWER;
				referencePosition = IN;
				break;
			case COLLECTION:
				referencePosition = COLLECTION;
				break;
			case GOING_TO_HIGH:
			case AT_HIGH:
				CURRENT_MAX_POWER = MAX_POWER;
				referencePosition = HIGH;
				break;
			case GOING_TO_MID:
			case AT_MID:
				CURRENT_MAX_POWER = MAX_POWER;
				referencePosition = MID;
				break;
			case GOING_TO_LOW:
			case AT_LOW:
				CURRENT_MAX_POWER = MAX_POWER;
				referencePosition = LOW;
				break;
			case AT_CAPPING:
				referencePosition = FOR_CAPPING;
				CURRENT_MAX_POWER = MAX_POWER_AT_CAP;
				break;
			case AT_CAPPING_LOW:
				referencePosition = FOR_CAPPING_TWO;
				CURRENT_MAX_POWER = MAX_POWER;
				break;
			case DEPOSITING:
			case GOING_IN:
				break;
		}
		System.out.println("reference position is " + referencePosition);
		double controllerCommand = slideController.stateReferenceCalculate(referencePosition, subsystemState().getPosition());
		slides.input(controllerCommand * CURRENT_MAX_POWER);
		error = slideController.getError();
		System.out.println("controller error is " + error + " controller setpoint is " + slideController.getReference());

	}

	public double getControllerError() {
		return error;
	}

	/**
	 * sets the state of our system
	 *
	 * @param state state as defined in the enum in the deposit class
	 */
	public void setState(Deposit.depositStates state) {
		this.state = state;
	}

	/**
	 * gets the position and velocity of the slide system
	 *
	 * @return position and velocity as state object
	 */
	@Override
	public State subsystemState() {
		return slides.getState();
	}

}
