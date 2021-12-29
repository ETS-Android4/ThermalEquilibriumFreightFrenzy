package homeostasis2.Utils;

import java.util.Arrays;

public class Vector {
	private double[] vec;
	private int size;
	public Vector(int size) {
		this.size = size;
		this.vec = new double[size];
		initializeToZero();
	}

	/**
	 * set the value of an array
	 * @param value the value we want to insert
	 * @param index index we are putting the value into
	 */
	public void set(double value, int index) {
		vec[index] = value;
	}

	public double get(int index) {
		return vec[index];
	}

	private void initializeToZero() {
		Arrays.fill(vec, 0);
	}




}
