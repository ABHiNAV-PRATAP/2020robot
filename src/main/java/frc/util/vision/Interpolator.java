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
import java.util.ArrayList;

public class Interpolator {
	ArrayList<ShooterProfile> profiles;

	public Interpolator() {
		profiles = new ArrayList<ShooterProfile>();	
	}

	public Interpolator(ArrayList<ShooterProfile> profilesToAdd) {
		profiles = new ArrayList<ShooterProfile>();
		for(int i=0; i<profilesToAdd.size(); i++) {
			this.add(profilesToAdd.get(i));
		}
	}

	public void add(ShooterProfile profileToAdd) {
		if(profiles.size() == 0) {
			profiles.add(profileToAdd);
		}
		else {
			for(int i=0; i<profiles.size(); i++) {
				if(profiles.get(i).getDistance() >= profileToAdd.getDistance()) {
					profiles.add(i, profileToAdd);
					return;
				}
			}
			profiles.add(profiles.size(), profileToAdd);
		}
	}

	public ShooterProfile interpolate(double distance) {
		for(int i=0; i<profiles.size() - 1; i++) {
			if(profiles.get(i).getDistance() == distance) {
				return profiles.get(i);
			}
			if(profiles.get(i).getDistance() < distance && profiles.get(i + 1).getDistance() > distance) {
				double tdy = profiles.get(i + 1).getTopShooterSpeed() - profiles.get(i).getTopShooterSpeed();
				double bdy = profiles.get(i + 1).getBottomShooterSpeed() - profiles.get(i).getBottomShooterSpeed();
				double dx = profiles.get(i + 1).getDistance() - profiles.get(i).getDistance();
				double tdydx = tdy / dx;
				double bdydx = bdy / dx;
				return new ShooterProfile(
					distance,
					profiles.get(i).getTopShooterSpeed() + tdydx * (distance - profiles.get(i).getDistance()),
					profiles.get(i).getBottomShooterSpeed() + bdydx * (distance - profiles.get(i).getDistance())
				);
			}
			if(profiles.get(i).getDistance() > distance) {
				return new ShooterProfile(
					distance,
					profiles.get(i).getTopShooterSpeed(),
					profiles.get(i).getBottomShooterSpeed()
				);
			}
		}
		if(profiles.get(profiles.size() - 1).getDistance() < distance) {
			return new ShooterProfile(
				distance,
				profiles.get(profiles.size() - 1).getTopShooterSpeed(),
				profiles.get(profiles.size() - 1).getBottomShooterSpeed()
			);
		}
		return new ShooterProfile(distance, 0, 0);
	}

	public String toString() {
		String res = "";
		for(int i=0; i<profiles.size(); i++) {
			res += profiles.get(i).toString() + "\n";
		}
		return res;
	}
}