package org.firstinspires.ftc.teamcode.subsystems.scoringMechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Filter.LowPassFilter;
import org.firstinspires.ftc.teamcode.subsystems.subsystem;

public class Bucket implements subsystem {

	protected double lowPassFilterGain = 0.999; // 0 < x < 1
	protected LowPassFilter filter = new LowPassFilter(lowPassFilterGain);
	protected Servo bucketServo;

	protected Deposit.depositStates state = Deposit.depositStates.IN;

	double lastPosition = 1000;

	double IN = 0.666;
	double OUT = 1;

	@Override
	public void init(HardwareMap hwmap) {
		bucketServo = hwmap.get(Servo.class, "bucket");
		setPosition(IN);
	}

	@Override
	public void initNoReset(HardwareMap hwmap) {
		init(hwmap);
	}


	/**
	 * called periodically, acts as a state management system for our subsystems logic.
	 */
	@Override
	public void update() {

		if (state.equals(Deposit.depositStates.DEPOSITING) || state.equals(Deposit.depositStates.COLLECTION)) {
			setPosition(OUT);
		} else {
			setPosition(IN);
		}


	}

	@Override
	public Object subsystemState() {
		return null;
	}

	/**
	 * set the servo position using lynx optimized servo calls
	 *
	 * @param position servo position
	 */
	protected void setPosition(double position) {

		if (position != lastPosition) {
			bucketServo.setPosition(position);
		}

		lastPosition = position;
	}


	/**
	 * because it is intended for an external subroutine to set the position, this state management method is required
	 *
	 * @param state the state we would like to go to
	 */
	public void setState(Deposit.depositStates state) {
		this.state = state;
	}
}
