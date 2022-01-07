package org.firstinspires.ftc.teamcode.subsystems.scoringMechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.subsystem;

import static org.firstinspires.ftc.teamcode.subsystems.scoringMechanisms.CapArm.ArmStates.IN;

public class CapArm implements subsystem {

	public Servo armServo;

	public String servoName = "caparm";  // TODO Benji fix this

	public final double inPosition = .1;
	public final double capPosition = .63;
	public final double actuallyPlaceCapOn = capPosition;
	public final double groundPosition = .85;
	public final double antennaPosition = 0.4;

	protected double previousPosition = 0;

	protected ArmStates state = IN;

	@Override
	public void init(HardwareMap hwmap) {
		armServo = hwmap.get(Servo.class, servoName);
	}

	@Override
	public void initNoReset(HardwareMap hwmap) {

	}

	@Override
	public void update() {
		switch (state) {
			case IN:
				setArmServoPosition(inPosition);
				break;
			case CAPPING:
				setArmServoPosition(capPosition);
				break;
			case CAP_DOWN:
				setArmServoPosition(actuallyPlaceCapOn);
				break;
			case ANTENNA:
				setArmServoPosition(antennaPosition);
				break;
			case DOWN:
				setArmServoPosition(groundPosition);
				break;
		}
	}

	@Override
	public ArmStates subsystemState() {
		return state;
	}

	/**
	 * lynx optimized servo setter
	 * @param position set the servo position
	 */
	protected void setArmServoPosition(double position) {
		if (position != previousPosition) {
			this.armServo.setPosition(position);
		}
		this.previousPosition = position;
	}

	public enum ArmStates {
		IN,
		CAPPING,
		CAP_DOWN,
		ANTENNA,
		DOWN;

		public ArmStates nextState() {
			switch (this) {
				case IN:
					return DOWN;
				case CAPPING:
					return CAP_DOWN;
				case CAP_DOWN:
				case ANTENNA:
					return IN;
				case DOWN:
					return CAPPING;
			}
			return IN;
		}

		public ArmStates previousState() {
			switch (this) {
				case ANTENNA:
				case IN:
				case DOWN:
					return IN;
				case CAPPING:
					return DOWN;
				case CAP_DOWN:
					return CAPPING;
			}
			return IN;
		}

	}

	/**
	 * set the state of the arm
	 * @param state of the arm
	 */
	public void setState(ArmStates state) {
		this.state = state;
	}

	public void nextState() {
		this.state = state.nextState();
	}

	public void previousState() {
		this.state = state.previousState();
	}
}
