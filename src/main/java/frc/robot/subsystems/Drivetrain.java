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

import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
  
  private WPI_TalonSRX leftDriveMaster;
  private WPI_VictorSPX leftDriveSlave;
  private WPI_TalonSRX rightDriveMaster;
  private WPI_VictorSPX rightDriveSlave;

  DifferentialDriveOdometry odometry;

  private AHRS gyro;

  /**
   * Creates a new Drivetrain.
   */
  public Drivetrain() {
    leftDriveMaster = new WPI_TalonSRX(Constants.kLeftDriveMasterPort);
    leftDriveSlave = new WPI_VictorSPX(Constants.kLeftDriveSlavePort);
    rightDriveMaster = new WPI_TalonSRX(Constants.kRightDriveMasterPort);
    rightDriveSlave = new WPI_VictorSPX(Constants.kRightDriveSlavePort);

    leftDriveMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    rightDriveMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);

    
    /*
    * Need to check encoder values and drivetrain inversions
    */

    // leftDriveMaster.setSensorPhase(false);
    // rightDriveMaster.setSensorPhase(true);
    rightDriveMaster.setInverted(true);
    rightDriveSlave.setInverted(true);


    gyro = new AHRS(Port.kMXP);

    leftDriveSlave.follow(leftDriveMaster);
    rightDriveSlave.follow(rightDriveMaster);

    resetEncoders();

    leftDriveMaster.setNeutralMode(NeutralMode.Brake);
    rightDriveMaster.setNeutralMode(NeutralMode.Brake);
    leftDriveSlave.setNeutralMode(NeutralMode.Brake);
    rightDriveSlave.setNeutralMode(NeutralMode.Brake);

    // leftDriveMaster.setNeutralMode(NeutralMode.Coast);
    // rightDriveMaster.setNeutralMode(NeutralMode.Coast);
    // leftDriveSlave.setNeutralMode(NeutralMode.Coast);
    // rightDriveSlave.setNeutralMode(NeutralMode.Coast);

  }

  public void arcadeDrive(double throttle, double turn) {
    setDriveMotors(throttle + turn, throttle - turn);
  }

  public void setDriveMotors(double leftValue, double rightValue) {
    leftDriveMaster.set(leftValue);
    rightDriveSlave.set(rightValue);
  }

  public void setDriveMotorVoltage(double leftVoltage, double rightVoltage)
  {
    leftDriveMaster.setVoltage(leftVoltage);
    rightDriveMaster.setVoltage(rightVoltage);
  }

  public void setDriveMotors(double value) {
    leftDriveMaster.set(value);
    rightDriveSlave.set(value);
  }

  public Rotation2d getHeading() {
    /**
    * 0 degrees / radians represents the robot angle when the robot is facing directly toward your 
    * opponent’s alliance station. As your robot turns to the left, 
    * your gyroscope angle should increase. By default, WPILib gyros exhibit 
    * the opposite behavior, so you should negate the gyro angle.
    */
    return Rotation2d.fromDegrees(-gyro.getYaw());
  }

  public double getHeadingAsAngle() {
    return getHeading().getDegrees();
  }

  public void stop() {
    leftDriveMaster.set(0);
    rightDriveMaster.set(0);
  }

  public void resetEncoders()
  {
    leftDriveMaster.setSelectedSensorPosition(0);
    rightDriveMaster.setSelectedSensorPosition(0);
  }

  public void resetGyro()
  {
    gyro.reset();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds()
  {
    return new DifferentialDriveWheelSpeeds((double) leftDriveMaster.getSelectedSensorVelocity() * 10 * Constants.kDistancePerTick, (double) rightDriveMaster.getSelectedSensorVelocity() * 10 * Constants.kDistancePerTick);
  }

  public Pose2d getPose()
  {
    return odometry.getPoseMeters();
  }

  /** 
   * @param poseMeters The position on the field the robot is at.
  */
  public void resetPose(Pose2d poseMeters)
  {
    odometry.resetPosition(poseMeters, getHeading());
    leftDriveMaster.setSelectedSensorPosition(0);
    rightDriveMaster.setSelectedSensorPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // odometry.update(getHeading(), leftDriveMaster.getSelectedSensorPosition() * Constants.kDistancePerTick, rightDriveMaster.getSelectedSensorPosition() * Constants.kDistancePerTick);
    // System.out.println("left: " + leftDriveMaster.getSelectedSensorPosition());
    // System.out.println("right: " + rightDriveMaster.getSelectedSensorPosition());
  }
}
