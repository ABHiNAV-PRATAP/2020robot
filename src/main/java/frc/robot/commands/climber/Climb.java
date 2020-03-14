/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.climber;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class Climb extends CommandBase {
  /**
   * Creates a new Climb.
   */

  Climber climber;
  DoubleSupplier winchSpeed;
  DoubleSupplier hookSpeed;

  public Climb(Climber climber, DoubleSupplier winchSpeed, DoubleSupplier hookSpeed) {
    addRequirements(climber);
    this.climber = climber;
    this.winchSpeed = winchSpeed;
    this.hookSpeed = hookSpeed;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climber.setClimberWinchSpeed(winchSpeed.getAsDouble());
    climber.setHookDeploySpeed(hookSpeed.getAsDouble() * 0.5);
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
