/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;
import com.playingwithfusion.CANVenom;
import com.playingwithfusion.CANVenom.BrakeCoastMode;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
  private WPI_TalonSRX leftDriveTalon;
  private CANVenom leftDriveVenom;
  private WPI_TalonSRX rightDriveTalon;
  private CANVenom rightDriveVenom;
  private SpeedControllerGroup leftMotors;
  private SpeedControllerGroup rightMotors;
  DifferentialDriveOdometry odometry;
  private AHRS gyro;
  private boolean flipped = false;
  /**
   * Creates a new Drivetrain.
   */
  public Drivetrain() {
    leftDriveTalon = new WPI_TalonSRX(Constants.kLeftDriveTalonPort);
    leftDriveVenom = new CANVenom(Constants.kLeftDriveVenomPort);
    rightDriveTalon = new WPI_TalonSRX(Constants.kRightDriveTalonPort);
    rightDriveVenom = new CANVenom(Constants.kRightDriveVenomPort);

    leftMotors = new SpeedControllerGroup(leftDriveTalon, leftDriveVenom);
    rightMotors = new SpeedControllerGroup(rightDriveTalon, rightDriveVenom);

    rightDriveTalon.setInverted(true);
    rightDriveVenom.setInverted(true);

    gyro = new AHRS(Port.kMXP);
    
    resetEncoders();
    resetGyro();
  }

  public void arcadeDrive(double throttle, double turn) {
    setDriveMotors(throttle + turn, throttle - turn);
  }

  public void setDriveMotors(double leftValue, double rightValue) {
    if (!flipped)
    {
      leftMotors.set(leftValue);
      rightMotors.set(rightValue);
    }
    if (flipped)
    {
      leftMotors.set(-rightValue);
      rightMotors.set(-leftValue);
    }
  }

  public void setDriveMotorVoltage(double leftVoltage, double rightVoltage)
  {
    if (!flipped)
    {
      leftMotors.set(leftVoltage);
      rightMotors.set(rightVoltage);
    }
    if (flipped)
    {
      leftMotors.set(-rightVoltage);
      rightMotors.set(-leftVoltage);
    }
  }

  public void setDriveMotors(double value) {
    if (!flipped)
    {
      leftMotors.set(value);
    }
    if (flipped)
    {
      rightMotors.set(-value);
    }
  }

  public Rotation2d getHeading() {
    /**
    * 0 degrees / radians represents the robot angle when the robot is facing directly toward your 
    * opponent's alliance station. As your robot turns to the left, 
    * your gyroscope angle should increase. By default, WPILib gyros exhibit 
    * the opposite behavior, so you should negate the gyro angle.
    */
    return Rotation2d.fromDegrees(-gyro.getAngle());//-gyro.getYaw());
  }

  public double getHeadingAsAngle() {
    return getHeading().getDegrees();
  }

  public void stop() {
    leftMotors.set(0);
    rightMotors.set(0);
  }

  public void resetEncoders()
  {
    leftDriveVenom.resetPosition();
    rightDriveVenom.resetPosition();
  }

  public void flipDT()
  {
    flipped = !flipped;
  }

  public void resetGyro()
  {
    gyro.reset();
  }

  // public DifferentialDriveWheelSpeeds getWheelSpeeds()
  // {
  //   return new DifferentialDriveWheelSpeeds((double) leftDriveMaster.getSelectedSensorVelocity() * 10 * Constants.kDistancePerTick, (double) rightDriveMaster.getSelectedSensorVelocity() * 10 * Constants.kDistancePerTick);
  // }
  //
  // public Pose2d getPose()
  // {
  //   return odometry.getPoseMeters();
  // }

  /** 
   * @param poseMeters The position on the field the robot is at.
  */
  public void resetPose(Pose2d poseMeters)
  {
    odometry.resetPosition(poseMeters, getHeading());
    // leftDriveMaster.setSelectedSensorPosition(0);
    // rightDriveMaster.setSelectedSensorPosition(0);
  }
  
  @Override
  public void periodic() {
    // System.out.println("Current heading: " + getHeadingAsAngle());
    // This method will be called once per scheduler run
    // odometry.update(getHeading(), leftDriveMaster.getSelectedSensorPosition() * Constants.kDistancePerTick, rightDriveMaster.getSelectedSensorPosition() * Constants.kDistancePerTick);
    // System.out.println("left: " + leftDriveMaster.getSelectedSensorPosition());
    // System.out.println("right: " + rightDriveMaster.getSelectedSensorPosition());
  }
}