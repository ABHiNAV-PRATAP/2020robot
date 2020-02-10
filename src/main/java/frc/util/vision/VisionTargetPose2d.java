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
public class VisionTargetPose2d {
	private double pitch;
	private double yaw;
	private double area;

	public VisionTargetPose2d() {
		pitch = 0;
		yaw = 0;
		area = 0;
	}

	public VisionTargetPose2d(double pitch, double yaw, double angle) {
		this.pitch = pitch;
		this.yaw = yaw;
		this.area = area;
	}

	public double getPitch() {
		return pitch;
	}

	public double getYaw() {
		return yaw;
	}

	public double getArea() {
		return area;
	}

	public String toString() {
		return "pitch: " + pitch + ", yaw: " + yaw + ", area: " + area;
	}
}