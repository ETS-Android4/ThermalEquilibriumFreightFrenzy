package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class DistanceSensorLocalization implements subsystem {

	ThreeWheelOdometry odom;

	String redSideDistanceSensorName = "distance1";

	DistanceSensor redSideSensor;

	protected double redSideMeasurement = 0;

	ElapsedTime timer = new ElapsedTime();

	protected final double frequencyHz = 5;

	protected final double DELAY = 1000.0 / frequencyHz;

	public DistanceSensorLocalization(ThreeWheelOdometry odom) {
		this.odom = odom;
	}

	@Override
	public void init(HardwareMap hwmap) {
		this.redSideSensor = hwmap.get(DistanceSensor.class, redSideDistanceSensorName);
	}

	@Override
	public void initNoReset(HardwareMap hwmap) {
		init(hwmap);

	}

	@Override
	public void update() {
		if (timer.milliseconds() < DELAY) return;
		timer.reset();
		readSensors();
		Dashboard.packet.put("distance",redSideMeasurement);
	}

	@Override
	public Object subsystemState() {
		return null;
	}

	protected void readSensors() {
		this.redSideMeasurement = redSideSensor.getDistance(DistanceUnit.INCH);
	}
}
