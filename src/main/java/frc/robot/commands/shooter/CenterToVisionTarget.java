/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shooter;

import java.lang.annotation.Target;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class CenterToVisionTarget extends CommandBase {
  /**
   * Creates a new CenterToVisionTarget.
   */

   private final double kP = 0, kD = 0;
   private final double tolerance = 5;
  PIDController centerPID = new PIDController(kP, 0, kD);
  DoubleSupplier yaw;
  Drivetrain drivetrain;
  private final double target = 0;

  double error = 0;

  public CenterToVisionTarget(Drivetrain drivetrain, DoubleSupplier yaw) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
    this.yaw = yaw;
    this.drivetrain = drivetrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    error = target - yaw.getAsDouble();

    if (error > tolerance)
    {
      drivetrain.setDriveMotors(-0.1, 0.1);
    }
    else if (error < -tolerance)
    {
      drivetrain.setDriveMotors(0.1, 0.1);
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
    System.out.println("done centering");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Math.abs(error) <= tolerance)
    {
      return true;
    }
    else
    {
    return false;
    }
  }
}
