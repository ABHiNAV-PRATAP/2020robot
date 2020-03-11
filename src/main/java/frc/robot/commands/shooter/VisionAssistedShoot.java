/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.Shooter;
import frc.util.MathUtil;
import frc.util.vision.ShooterProfile;

public class VisionAssistedShoot extends CommandBase {
  private final Shooter shooter;
  private final Pneumatics pneumatics;
  /**
   * Creates a new VisionAssistedShoot.
   */
  public VisionAssistedShoot(Shooter shooter, Pneumatics pneumatics) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    this.pneumatics = pneumatics;
    addRequirements(shooter, pneumatics);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pneumatics.TurnOffSolenoid();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("Distance to target:" + shooter.getDistanceToTarget());
    ShooterProfile currentProfile = shooter.getShooterProfileFromInterpolator(shooter.getDistanceToTarget());
    double tsp = currentProfile.getTopShooterSpeed();
    double bsp = currentProfile.getBottomShooterSpeed();
      double tv = shooter.getTopVelocity();
      double bv = shooter.getBottomVelocity();
      if(MathUtil.withinTolerance(tv, tsp, 3)) {
        shooter.servoOpen();
        pneumatics.OpenSolenoid();
        //pneumatics.TurnOffSolenoid();
      }
      shooter.tpid.setSetpoint(tsp);
      shooter.bpid.setSetpoint(bsp);
      double calctop = shooter.tpid.calculate(tv);
      double calcBot = shooter.bpid.calculate(bv);
      // System.out.println(calcBot + shooter.bff.calculate(bsp));
      shooter.setTopMotorVoltage(calctop + shooter.tff.calculate(tsp));
      shooter.setBottomMotorVoltage(calcBot + shooter.bff.calculate(bsp));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setTopMotorVoltage(0);
    shooter.setBottomMotorVoltage(0);
    pneumatics.TurnOffSolenoid();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
