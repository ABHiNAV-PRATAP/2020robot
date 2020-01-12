/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
  private WPI_TalonSRX leftDriveMaster;
  private WPI_TalonSRX leftDriveSlave;
  private WPI_TalonSRX rightDriveMaster;
  private WPI_TalonSRX rightDriveSlave;

  private AHRS gyro;

  /**
   * Creates a new Drivetrain.
   */
  public Drivetrain() {
    leftDriveMaster = new WPI_TalonSRX(Constants.kLeftDriveMasterPort);
    leftDriveSlave = new WPI_TalonSRX(Constants.kLeftDriveSlavePort);
    rightDriveMaster = new WPI_TalonSRX(Constants.kRightDriveMasterPort);
    rightDriveSlave = new WPI_TalonSRX(Constants.kRightDriveSlavePort);

    rightDriveMaster.setInverted(true);
    rightDriveSlave.setInverted(true);

    gyro = new AHRS(Port.kMXP);

    leftDriveSlave.follow(leftDriveMaster);
    rightDriveSlave.follow(rightDriveMaster);

    gyro.reset();
  }

  public void setDriveMotors(double leftValue, double rightValue) {
    leftDriveMaster.set(leftValue);
    rightDriveSlave.set(rightValue);
  }

  public void setDriveMotors(double value) {
    leftDriveMaster.set(value);
    rightDriveSlave.set(value);
  }

  public double getHeading() {
    return gyro.getAngle();
  }

  public void stop() {
    leftDriveMaster.set(0);
    rightDriveMaster.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
