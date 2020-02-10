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
public class ShooterProfile {
	private double distance;
	private double topShooterSpeed;
	private double bottomShooterSpeed;

	public ShooterProfile(double distance, double topShooterSpeed, double bottomShooterSpeed) {
		this.distance = distance;
		this.topShooterSpeed = topShooterSpeed;
		this.bottomShooterSpeed = bottomShooterSpeed;
	}

	public double getDistance() {
		return this.distance;
	}

	public double getTopShooterSpeed() {
		return this.topShooterSpeed;
	}

	public double getBottomShooterSpeed() {
		return this.bottomShooterSpeed;
	}

	public String toString() {
		String res = distance + " : " + topShooterSpeed + ", " + bottomShooterSpeed;
		return res;
	}
}
