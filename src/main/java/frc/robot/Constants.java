/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    // Drivetrain Motor Ports
    public static final int kLeftDriveMasterPort = 1;
    public static final int kLeftDriveSlavePort = 2;
    public static final int kRightDriveMasterPort = 3;
    public static final int kRightDriveSlavePort = 4;

    // OI
    public static final int kDriveJoystickPort = 0;

    // Drivetrain Control
    public static final double kDriveMaxAxis = 1;
    public static final double kDriveMinAxis = 1;
    public static final double kDriveLowRange = 0.2;
    public static final double kDriveHighRange = 1;
    public static final double kTicksPerRotation = 4096;

    public static final double kDriveTurnP = 0.0;
    public static final double kDriveTurnI = 0.0;
    public static final double kDriveTurnD = 0.0;
    public static final double kMaxTurnRateDegPerS = 0.0;
    public static final double kMaxTurnAccelerationDegPerSSquared = 0.0;
    public static final double kTurnToleranceDeg = 0.0;
    public static final double kTurnRateToleranceDegPerS = 0.0;

    // Drivetrain Characterization & Ramsete Constants
    public static final double ksVolts = 0;
    public static final double kvVoltSecondsPerMeter = 0;
    public static final double kaVoltSecondsSquaredPerMeter = 0;
    public static final double kPDriveVelocity = 0;

    public static final double kTrackwidthMeters = 0; // horizontal distance between the wheels
    public static final double kWheelDiameterMeters = 0;
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);    
    public static final double kMaxSpeedMetersPerSecond = 0;
    public static final double kMaxAccelerationMetersPerSecondSquared = 0;
    public static final double kRamseteBeta = 2;
    public static final double kRamseteZeta = 0.7;
    public static final double kDistancePerTick = Constants.kWheelDiameterMeters * Math.PI / kTicksPerRotation;

    // Universal Constants
    public static final double kDT = 0.02; // 20 ms = 0.02 s

}
