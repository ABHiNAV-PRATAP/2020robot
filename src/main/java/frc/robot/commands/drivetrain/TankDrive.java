/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivetrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class TankDrive extends CommandBase {
  /**
   * Creates a new TankDrive.
   */

  Drivetrain drive;
  DoubleSupplier left, right;

  public TankDrive(Drivetrain drive, DoubleSupplier leftDrive, DoubleSupplier rightDrive) {
    addRequirements(drive);
    this.drive = drive;
    this.right = rightDrive;
    this.left = leftDrive;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.stop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drive.setDriveMotors(left.getAsDouble(), right.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
