package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.geometry.Vector3D;
import org.firstinspires.ftc.teamcode.utils.Quadratic;
import org.firstinspires.ftc.teamcode.utils.Regression;
import org.firstinspires.ftc.teamcode.utils.SizedStack;

public class IMUOdom implements subsystem {

	protected boolean hasRun = false;
	protected drivetrain drive;
	protected Vector3D imuPos ;
	protected Vector3D imuPosDelta;
	protected Vector3D accel;
	SizedStack<Vector3D> accels = new SizedStack<>(3);
	SizedStack<Double> dts = new SizedStack<>(3);
	ElapsedTime timer;
	private int callCounter = 0;


	public IMUOdom (drivetrain drive) {
		// initialize our accel stacks to be 0
		accels.add(new Vector3D());
		accels.add(new Vector3D());
		accels.add(new Vector3D());
		dts.add(0.0);
		dts.add(0.0);
		dts.add(0.0);
		timer = new ElapsedTime();
		this.drive = drive;

	}

	@Override
	public void init(HardwareMap hwmap) {

		imuPos = new Vector3D();

	}

	@Override
	public void initNoReset(HardwareMap hwmap) {
		imuPos = drive.getRobotPosition();
	}



	public void updateWithAccel(Vector3D accel) {
		this.accel = accel;
		update();
	}
	@Override
	public void update() {
		if (!hasRun) {
			timer.reset();
			hasRun = true;
		}
		// rotate the accel values by the rotation vector
		// push accel and delta time to the stack
		dts.push(timer.seconds());
		timer.reset();
		accels.push(accel.rotateBy(drive.robotPosition.getAngleRadians()));
		if (callCounter % 3 == 0)
		{
			double[] x = {accels.get(0).getX(), accels.get(1).getX(), accels.get(2).getX()};
			double[] y = {accels.get(0).getY(),accels.get(1).getY(),accels.get(2).getY()};
			double[] dtArray = {dts.get(0), dts.get(0) + dts.get(1), dts.get(0) + dts.get(1) + dts.get(2)};

			// run quadratic regression on the data
			Quadratic xRegression = Regression.quadraticRegression(dtArray,x);
			Quadratic yRegression = Regression.quadraticRegression(dtArray,y);
			// sum of dt's
			double secondDt = dts.get(0) + dts.get(1);
			double endDt = secondDt + dts.get(2);
			// calculate the double integral from the time of the second point to the third point
			// this double integral is the change in position from this time step to the last

			double xDelta = xRegression.rangedDoubleIntegral(0,endDt);
			double yDelta = yRegression.rangedDoubleIntegral(0,endDt);
			imuPosDelta = new Vector3D(xDelta,yDelta,0).scale(1.0/3.0,false); // we are going to reuse the same measurement three times, this is cringe so
			imuPos = imuPos.add(imuPosDelta);

		}
		callCounter++;
	}



	@Override
	public Vector3D subsystemState() {
		return imuPos;
	}

	public Vector3D getStateDelta() {
		return imuPosDelta;
	}
}
