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
import frc.robot.subsystems.Shooter;
import frc.util.MathUtil;

public class BangBangRotate extends CommandBase {
  private final Drivetrain drivetrain;
  private final Shooter shooter;
  private double setpointValue;
  private double error;

  /**
   * Creates a new BangBangRotate.
   */
  public BangBangRotate(Drivetrain drivetrain, Shooter shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.shooter = shooter;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    setpointValue = drivetrain.getHeadingAsAngle() - shooter.getHorizontalOffset();
    // System.out.println("Setpoint: " + setpointValue);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // System.out.println("Rotation Setpoint: " + setpointValue);
    // System.out.println("current heading: " + drivetrain.getHeadingAsAngle());
    error = drivetrain.getHeadingAsAngle() - setpointValue;
    drivetrain.arcadeDrive(0, 0.3 * MathUtil.sign(error));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return MathUtil.withinTolerance(drivetrain.getHeadingAsAngle(), setpointValue, 7);
  }
}
