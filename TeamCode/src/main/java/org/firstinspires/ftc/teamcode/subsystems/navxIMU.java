package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.geometry.Vector3D;


import kauailabs.navx.ftc.AHRS;

import static org.firstinspires.ftc.teamcode.utils.utils.normalizeAngleRR;

public class navxIMU implements subsystem {

	private AHRS navx_device;
	private double initialAngle = 0;
	private double angleEstimate = 0;
	private Vector3D accel = new Vector3D();
	ElapsedTime timer = new ElapsedTime();
	boolean firstRun = true;

	@Override
	public void init(HardwareMap hwmap) {
		initNoReset(hwmap);
		navx_device.zeroYaw();
		initialAngle = normalizeAngleRR(Math.toRadians(navx_device.getFusedHeading()));
		while (navx_device.isCalibrating() && !navx_device.isMagnetometerCalibrated()) {
			System.out.println("is navX calibrating? " +  navx_device.isCalibrating() + " is navX mag calibrated? " + navx_device.isMagnetometerCalibrated());
		}
	}

	@Override
	public void initNoReset(HardwareMap hwmap) {
		navx_device = AHRS.getInstance(hwmap.get(NavxMicroNavigationSensor.class, "navx"),
				AHRS.DeviceDataType.kProcessedData);
	}

	@Override
	public void update() {
		if (firstRun) {
			firstRun = false;
			timer.reset();
		}
		angleEstimate = normalizeAngleRR(Math.toRadians(-navx_device.getFusedHeading()));
		angleEstimate = normalizeAngleRR(angleEstimate + initialAngle);
		accel = new Vector3D(navx_device.getWorldLinearAccelX(),navx_device.getWorldLinearAccelY(), navx_device.getWorldLinearAccelZ(),0,0);
		timer.reset();
	}

	/**
	 * set the initial angle
	 * @param initialAngle the initial angle
	 */
	public void setInitialAngle(double initialAngle) {
		this.initialAngle = initialAngle;
	}

	public void setInitialAngle(Vector3D pos) {
		this.initialAngle = pos.getAngleRadians();
	}
	@Override
	public Vector3D subsystemState() {
		return new Vector3D(0,0,angleEstimate);
	}

	public boolean isRotating() {
		return navx_device.isRotating();
	}


	public Vector3D getAccel() {
		return accel;
	}


	public void stop() {
		navx_device.close();
	}
}
