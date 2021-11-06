package org.firstinspires.ftc.teamcode.subsystems.scoringMechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.basedControl.basedControl;
import org.firstinspires.ftc.teamcode.basedControl.controllerCoefficients;
import org.firstinspires.ftc.teamcode.subsystems.subsystem;

import java.util.ArrayList;

import homeostasis.systems.DcMotorPlant;
import homeostasis.utils.state;

public class slides implements subsystem {

	protected basedControl slideController = new basedControl(controllerCoefficients.slideCoefficients, 0, 5, 0.1, 2);
	protected DcMotorPlant slides;
	protected DcMotorEx left;
	protected DcMotorEx right;

	protected deposit.depositStates state = deposit.depositStates.IN;

	protected double IN = 0;
	protected double COLLECTION = 0;
	protected double LOW = 100;
	protected double MID = 100;
	protected double HIGH = 200;

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
		if (state.equals(deposit.depositStates.DISARMED)) {
			slides.input(0);
			return;
		}

		switch (state) {

			case IN:
				referencePosition = IN;
				break;
			case COLLECTION:
				referencePosition = COLLECTION;
				break;
			case AT_HIGH:
				referencePosition = HIGH;
				break;
			case AT_MID:
				referencePosition = MID;
				break;
			case AT_LOW:
				referencePosition = LOW;
				break;
			case DEPOSITING:
				// theoretically no change
				break;
		}
		System.out.println("reference position is " + referencePosition);
		double controllerCommand = slideController.stateReferenceCalculate(referencePosition, subsystemState().getPosition());
		slides.input(controllerCommand);
		error = slideController.getError();
		System.out.println("controller error is " + error + " controller setpoint is " + slideController.getError());


	}

	public double getControllerError() {
		return error;
	}

	/**
	 * sets the state of our system
	 *
	 * @param state state as defined in the enum in the deposit class
	 */
	public void setState(deposit.depositStates state) {
		this.state = state;
	}

	/**
	 * gets the position and velocity of the slide system
	 *
	 * @return position and velocity as state object
	 */
	@Override
	public state subsystemState() {
		return slides.getState();
	}

}
