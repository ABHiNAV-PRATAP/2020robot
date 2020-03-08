/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
public class LimelightVision extends SubsystemBase {
  /**
   * Creates a new Vision.
   */
  private final NetworkTable limelight =  NetworkTableInstance.getDefault().getTable("limelight");
 

  public double getHorizontalOffset() {
    return limelight.getEntry("tx").getDouble(0);
  }
  public double getVerticalOffset() {
    return limelight.getEntry("ty").getDouble(0);
  }
  public double getArea() {
    return limelight.getEntry("ta").getDouble(0);
  }
  public double getSkew() {
    return limelight.getEntry("ts").getDouble(0);
  }
  public double getDistance() {
    return (Constants.kTargetHeight - Constants.kCameraHeight) / (Math.tan(Constants.kCameraAngle + getVerticalOffset()));
  }
  @Override
  public void periodic() {
    System.out.println("Yaw angle to target: " + getHorizontalOffset());
    // System.out.println("Distance to target: " + getDistance());
  }
}