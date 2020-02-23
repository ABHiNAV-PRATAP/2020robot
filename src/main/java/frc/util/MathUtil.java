/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.util;

/**
 * Utility class for mathematical functions used on the robot
 */
public class MathUtil {
	/**
	 * Normalize value within a given range
	 * @param min Minimum value in input range
	 * @param max Maximum value in input range
	 * @param a Minimum value in output range
	 * @param b Maximum value in output range
	 * @param x Value to be normalized
	 * @return
	 */
    public static double normalize(double min, double max, double a, double b, double x) {
		return (((b-a) * (x-min)) / (max-min)) + a;
	}

	public static double constrain(double min, double max, double x) {
		if(x < min) {
			return min;
		}
		if (x > max) {
			return max;
		}
		return x;
	}

	public static boolean withinTolerance(double a, double b, double tolerance) {
		return Math.abs(a - b) <= tolerance;
	}

	public static double sign(double a) {
		return a / Math.abs(a);
	}
}
