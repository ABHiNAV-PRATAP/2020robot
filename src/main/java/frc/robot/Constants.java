/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    // Drivetrain Motor Ports
    public static int kLeftDriveMasterPort = 0;
    public static int kLeftDriveSlavePort = 1;
    public static int kRightDriveMasterPort = 2;
    public static int kRightDriveSlavePort = 3;

    // OI
    public static int kDriveJoystickPort = 0;

    // Drivetrain Control
    public static double kDriveMaxAxis = 1;
    public static double kDriveMinAxis = 1;
    public static double kDriveLowRange = 0.2;
    public static double kDriveHighRange = 1;
}
