package org.firstinspires.ftc.teamcode.subsystems.scoringMechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.subsystem;

public class virtual4Bar implements subsystem {

	protected final double IN = 0;
	protected final double LOW = 0.5;
	protected final double MID = 0.5;
	protected final double HIGH = 0.5;
	protected Servo left;
	protected Servo right;
	protected deposit.depositStates state = deposit.depositStates.IN;
	protected double lastPosition = 1000;

	ElapsedTime timer = new ElapsedTime();

	@Override
	public void init(HardwareMap hwmap) {
		left = hwmap.get(Servo.class, "leftV4B");
		right = hwmap.get(Servo.class, "rightV4B");
		right.setDirection(Servo.Direction.REVERSE); // TODO this might be wrong

		setPosition(IN);

	}

	@Override
	public void initNoReset(HardwareMap hwmap) {
		init(hwmap);
	}

	@Override
	public void update() {

		switch (state) {
			case DISARMED:
				setPosition(IN);
				System.out.println("disarmed");
				break;
			case IN:
			case COLLECTION:
				setPosition(IN);
				break;
			case AT_HIGH:
				setPosition(HIGH);
				break;
			case AT_MID:
				setPosition(MID);
				break;
			case AT_LOW:
				setPosition(LOW);
				break;
			case DEPOSITING:
				// theoretically nothing should change?
				break;
		}
		timer.reset();

	}

	@Override
	public Object subsystemState() {
		return null;
	}


	public void setState(deposit.depositStates state) {
		this.state = state;
	}

	/**
	 * set the 4 bar angle while optimizing lynx calls
	 *
	 * @param position the -1 < theta < 1 angle of the virtual 4 bar
	 */
	protected void setPosition(double position) {

		if (position != lastPosition) {
			left.setPosition(position);
			right.setPosition(position);
		}

		lastPosition = position;
	}
}
