package org.firstinspires.ftc.teamcode.subsystems.scoringMechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.subsystem;

public class BucketFlip implements subsystem {

	String servoLName = "BuckLeft";
	String servoRName = "BucketRight";

	Servo leftServo;
	Servo rightServo;

	public final double COLLECTION = 0;
	public final double OUT = 0.6666666667;
	public final double REST = 0.1;

	protected double previousPosition = 100;

	Deposit.depositStates state = Deposit.depositStates.IN;

	@Override
	public void init(HardwareMap hwmap) {

		leftServo = hwmap.get(Servo.class, servoLName);
		rightServo = hwmap.get(Servo.class, servoRName);

		rightServo.setDirection(Servo.Direction.REVERSE);
		leftServo.setDirection(Servo.Direction.FORWARD);


	}

	@Override
	public void initNoReset(HardwareMap hwmap) {
		init(hwmap);
	}

	@Override
	public void update() {

		switch (state) {
			case DISARMED:
			case GOING_IN:
			case IN:
				setServoPosition(REST);
				break;
			case COLLECTION:
				setServoPosition(COLLECTION);
				break;
			case GOING_TO_HIGH:
			case DEPOSITING:
			case AT_LOW:
			case AT_MID:
			case AT_HIGH:
			case GOING_TO_LOW:
			case GOING_TO_MID:
				setServoPosition(OUT);
				break;
		}


	}

	@Override
	public Deposit.depositStates subsystemState() {
		return state;
	}

	public void setState(Deposit.depositStates state) {
		this.state = state;
	}

	protected void setServoPosition(double position) {
		if (position != previousPosition) {
			this.leftServo.setPosition(position);
			this.rightServo.setPosition(position);
		}
		previousPosition = position;
	}

}
