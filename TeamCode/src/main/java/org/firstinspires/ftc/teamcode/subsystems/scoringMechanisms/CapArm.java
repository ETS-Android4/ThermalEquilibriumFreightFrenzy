package org.firstinspires.ftc.teamcode.subsystems.scoringMechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.subsystem;

import static org.firstinspires.ftc.teamcode.subsystems.scoringMechanisms.CapArm.ArmStates.IN;

public class CapArm implements subsystem {

	public Servo armServo;

	public String servoName = "BenjiNameThisLol";  // TODO Benji fix this

	public final double inPosition = 0;
	public final double capPosition = 0;
	public final double groundPosition = 0;

	public double previousPosition = 0;

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
		DOWN
	}

	/**
	 * set the state of the arm
	 * @param state of the arm
	 */
	public void setState(ArmStates state) {
		this.state = state;
	}
}
