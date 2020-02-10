/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.util.vision;

/**
 * Add your docs here.
 */
public class VisionTargetPose3d {
	private double x;
	private double y;
	private double angle;

	public VisionTargetPose3d() {
		x = 0;
		y = 0;
		angle = 0;
	}

	public VisionTargetPose3d(Double[] targetPose) {
		x = targetPose[0];
		y = targetPose[1];
		angle = targetPose[2];
	}

	public double getX() {
		return x;
	}

	public double getY() {
		return y;
	}

	public double getAngle() {
		return angle;
	}

	public String toString() {
		return "x: " + x + ", y: " + y + ", angle: " + angle;
	}
}
