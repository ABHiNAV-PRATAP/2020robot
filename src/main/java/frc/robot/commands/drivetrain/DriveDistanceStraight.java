/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DriveDistanceStraight extends CommandBase {

  private final Drivetrain drivetrain;
  private double angleSetpoint;
  private double turnError;

  private double kTurnP = 0.02;
  /**
   * Creates a new DriveDistanceStraight.
   */
  public DriveDistanceStraight(Drivetrain drivetrain) {
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    angleSetpoint = drivetrain.getHeadingAsAngle();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    turnError = angleSetpoint + drivetrain.getHeadingAsAngle();
    double turnOutput = kTurnP * turnError;
    System.out.println("Current heading: " + drivetrain.getHeadingAsAngle());
    drivetrain.arcadeDrive(0.2, turnOutput);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
