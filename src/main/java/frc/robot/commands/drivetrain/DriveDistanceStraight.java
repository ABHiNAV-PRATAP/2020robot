/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.util.MathUtil;

public class DriveDistanceStraight extends CommandBase {

  private final Drivetrain drivetrain;
  private final Intake intake;
  private double angleSetpoint;
  private double distanceSetpoint;
  private double turnError;

  private double direction;

  private double currentDistance;

  private double distanceError;

  private double kTurnP = 0.02;
  private double kDriveP = 0.015;
  private double kDriveFF = 0.0825;
  /**
   * Creates a new DriveDistanceStraight.
   */
  public DriveDistanceStraight(Drivetrain drivetrain, Intake intake, double distanceSetpoint /* in inches */) {
    this.drivetrain = drivetrain;
    this.distanceSetpoint = distanceSetpoint;
    this.intake = intake;
    direction = MathUtil.sign(distanceSetpoint);
    addRequirements(drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    angleSetpoint = drivetrain.getHeadingAsAngle();
    drivetrain.resetEncoders();
    System.out.println("Distance setpoint: " + distanceSetpoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.setValue(1);
    distanceError = distanceSetpoint - (drivetrain.venomTicksToInches(drivetrain.getLeftEncoderDistance()) + drivetrain.venomTicksToInches(drivetrain.getRightEncoderDistance()) / 2.0);
    turnError = angleSetpoint - drivetrain.getHeadingAsAngle();
    // System.out.println("Error:" + turnError);
    double turnOutput = kTurnP * turnError;
    // double throttleOutput = kDriveP * distanceError;
    currentDistance = drivetrain.venomTicksToInches(drivetrain.getLeftEncoderDistance());// + drivetrain.venomTicksToInches(drivetrain.getRightEncoderDistance())) / 2.0;
    System.out.println("Current distance: " + currentDistance);
    // System.out.println("Current Distance: " + currentDistance);
    // distanceError = MathUtil.constrain(-1, 1, distanceError);
    drivetrain.arcadeDrive(0.2*direction, -turnOutput);
  } 

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return MathUtil.withinTolerance(distanceSetpoint, currentDistance, 2);
    // return false;
  }
}
