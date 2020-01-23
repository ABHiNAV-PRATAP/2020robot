/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivetrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.util.MathUtil;

public class ArcadeDrive extends CommandBase {
  private final Drivetrain drivetrain;
  private final DoubleSupplier throttleSupplier;
  private final DoubleSupplier turnSupplier;
  private final DoubleSupplier accelerationSupplier;

  /**
   * Creates a new ArcadeDrive.
   */
  public ArcadeDrive(Drivetrain drivetrain, DoubleSupplier throttleSupplier, DoubleSupplier turnSupplier, DoubleSupplier accelerationSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.throttleSupplier = throttleSupplier;
    this.turnSupplier = turnSupplier;
    this.accelerationSupplier = accelerationSupplier;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.stop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double throttle = throttleSupplier.getAsDouble();
    double turn = turnSupplier.getAsDouble();
    double speedMultiplier = MathUtil.normalize(Constants.kDriveMaxAxis, Constants.kDriveMinAxis, Constants.kDriveLowRange, Constants.kDriveHighRange, accelerationSupplier.getAsDouble());

    drivetrain.arcadeDrive(throttle * speedMultiplier, turn * speedMultiplier);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
