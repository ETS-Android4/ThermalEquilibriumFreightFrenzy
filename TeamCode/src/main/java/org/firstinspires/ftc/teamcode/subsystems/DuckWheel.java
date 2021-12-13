package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DuckWheel implements subsystem{

	protected CRServo leftServo;
	protected CRServo rightServo;

	protected DuckWheelState state = DuckWheelState.OFF;

	protected double previousPower = 0;
	protected final double TURN_POWER = -.5;

	@Override
	public void init(HardwareMap hwmap) {
		this.leftServo = hwmap.get(CRServo.class, "DuckWheelL");
		this.rightServo = hwmap.get(CRServo.class, "DuckWheelR");
	}

	@Override
	public void initNoReset(HardwareMap hwmap) {
		init(hwmap);
	}

	@Override
	public void update() {
		switch (state) {
			case ON:
				setServoPower(TURN_POWER);
				break;
			case OTHER_ON:
				setServoPower(-TURN_POWER);
			case OFF:
				setServoPower(0);
				break;
		}
	}

	public void setState(DuckWheelState state) {
		this.state = state;
	}

	@Override
	public DuckWheelState subsystemState() {
		return state;
	}

	public void setServoPower(double power) {
		if (power != previousPower) {
			leftServo.setPower(power);
			rightServo.setPower(power);
		}
		previousPower = power;
	}

	public enum DuckWheelState {
		ON,
		OTHER_ON,
		OFF
	}
}
