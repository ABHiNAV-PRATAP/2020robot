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
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
  
  private WPI_TalonSRX leftDriveMaster;
  private WPI_TalonSRX leftDriveSlave;
  private WPI_TalonSRX rightDriveMaster;
  private WPI_TalonSRX rightDriveSlave;

  DifferentialDrive drive;
  DifferentialDriveOdometry odometry;

  private AHRS gyro;

  /**
   * Creates a new Drivetrain.
   */
  public Drivetrain() {
    leftDriveMaster = new WPI_TalonSRX(Constants.kLeftDriveMasterPort);
    leftDriveSlave = new WPI_TalonSRX(Constants.kLeftDriveSlavePort);
    rightDriveMaster = new WPI_TalonSRX(Constants.kRightDriveMasterPort);
    rightDriveSlave = new WPI_TalonSRX(Constants.kRightDriveSlavePort);

    leftDriveMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    rightDriveMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);

    // leftDriveMaster.setSensorPhase(false);
    // rightDriveMaster.setSensorPhase(true);

    rightDriveMaster.setInverted(true);
    rightDriveSlave.setInverted(true);

    gyro = new AHRS(Port.kMXP);

    leftDriveSlave.follow(leftDriveMaster);
    rightDriveSlave.follow(rightDriveMaster);

    resetGyro();
    resetEncoders();

    leftDriveMaster.setNeutralMode(NeutralMode.Coast);
    rightDriveMaster.setNeutralMode(NeutralMode.Coast);
    leftDriveSlave.setNeutralMode(NeutralMode.Coast);
    rightDriveSlave.setNeutralMode(NeutralMode.Coast);

    drive = new DifferentialDrive(leftDriveMaster, rightDriveMaster);

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

  public void arcadeDrive(double xSpeed, double zRotation)
  {
    drive.arcadeDrive(xSpeed, zRotation, false); // Does NOT square the inputs at low speeds to decrease sensitivity
  }

  public void setDriveMotors(double value) {
    leftDriveMaster.set(value);
    rightDriveSlave.set(value);
  }

  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(-gyro.getAngle());
    
    // return Math.IEEEremainder(gyro.getAngle(), 360) * (Constants.kGyroReversed ? -1.0 : 1.0); // The reaminder is the angle since heading angle accumulates past 360 degrees i.e. 365 degrees, 500 degrees
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    odometry.update(getHeading(), leftDriveMaster.getSelectedSensorPosition() * Constants.kDistancePerTick, rightDriveMaster.getSelectedSensorPosition() * Constants.kDistancePerTick);
  }
}
