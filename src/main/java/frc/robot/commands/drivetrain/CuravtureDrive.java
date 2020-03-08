/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivetrain;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.util.MathUtil;

public class CuravtureDrive extends CommandBase {
  /**
   * Creates a new CuravtureDrive.
   */

  Drivetrain dt;
  DoubleSupplier xSpeed;
  DoubleSupplier zRotation;
  BooleanSupplier isQuickTurn;
  DoubleSupplier accelerationSupplier;

  public CuravtureDrive(Drivetrain dt, DoubleSupplier xSpeed, DoubleSupplier zRotation, BooleanSupplier isQuickTurn, DoubleSupplier accelerationSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(dt);
    this.dt = dt;
    this.xSpeed = xSpeed;
    this.zRotation = zRotation;
    this.isQuickTurn = isQuickTurn;
    this.accelerationSupplier = accelerationSupplier;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speedMultiplier = accelerationSupplier.getAsDouble();
    double throttle = xSpeed.getAsDouble();
    // double turn = Math.pow(zRotation.getAsDouble(), 2) * Math.signum(zRotation.getAsDouble());
    double turn = zRotation.getAsDouble();
    
    speedMultiplier = MathUtil.normalize(1, -1, 0.2, 1, speedMultiplier);
    // System.out.println("Speed Multiplier " + speedMultiplier);
    // System.out.println("X " + turn);
    // System.out.println("Y " + throttle);

    dt.CurvatureDrive(throttle * speedMultiplier, turn, isQuickTurn.getAsBoolean());
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
