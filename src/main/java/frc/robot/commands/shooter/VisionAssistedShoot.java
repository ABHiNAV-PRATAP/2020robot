/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;
import frc.util.vision.ShooterProfile;

public class VisionAssistedShoot extends CommandBase {
  private final Shooter shooter;

  private double topShooterSpeed;
  private double bottomShooterSpeed;
  /**
   * Creates a new VisionAssistedShoot.
   */
  public VisionAssistedShoot(Shooter shooter) {
    this.shooter = shooter;
    topShooterSpeed = 0;
    bottomShooterSpeed = 0;
    addRequirements(shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // System.out.println("Initializing VisionAssistedShoot command");
    // ShooterProfile currentProfile = shooter.getShooterProfileFromInterpolator(shooter.getXToTarget());
    // topShooterSpeed = currentProfile.getTopShooterSpeed();
    double x = shooter.getXToTarget();
    topShooterSpeed = -1182.5523*Math.pow(x, 3) + 13205.1580*Math.pow(x, 2) - 49074.4837*x + 60713.2697;
    bottomShooterSpeed = 90;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.tpid.setSetpoint(topShooterSpeed);
    shooter.bpid.setSetpoint(bottomShooterSpeed);
    double calctop = shooter.tpid.calculate(shooter.getTopVelocity());
    double calcBot = shooter.bpid.calculate(shooter.getBottomVelocity());
    shooter.setTopMotorVoltage(calctop + shooter.tff.calculate(topShooterSpeed));
    shooter.setBottomMotorVoltage(calcBot + shooter.bff.calculate(bottomShooterSpeed));
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
