/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.VisionLEDs;
import frc.util.MathUtil;
import frc.util.vision.ShooterProfile;

public class TeleopVisionAssistedShoot extends CommandBase {
  private final Shooter shooter;
  private final VisionLEDs leds;

  private double x;

  private double timeout;
  private int ctr;
  private double topShooterSpeed;
  private double bottomShooterSpeed;
  /**
   * Creates a new VisionAssistedShoot.
   */
  public TeleopVisionAssistedShoot(Shooter shooter, VisionLEDs leds) {
    this.shooter = shooter;
    this.leds = leds;

    addRequirements(shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // this.x = shooter.getTargetPose3d().getX();
    x = shooter.getXToTarget();

    // shooter.setTargetPose2d(shooter.getTargetPose2d());
    // shooter.setTargetPose3d(shooter.getTargetPose3d());
    ShooterProfile currentProfile = shooter.getShooterProfileFromInterpolator(x);

    topShooterSpeed = currentProfile.getTopShooterSpeed();
    bottomShooterSpeed = currentProfile.getBottomShooterSpeed();
    System.out.println("x: " + x);
    System.out.println("Top Shooter speed: " + topShooterSpeed);
    System.out.println("Bottom Shooter speed: " + bottomShooterSpeed);
    
    // System.out.println("Initializing VisionAssistedShoot command");
    // ShooterProfile currentProfile = shooter.getShooterProfileFromInterpolator(shooter.getXToTarget());
    // topShooterSpeed = currentProfile.getTopShooterSpeed();
    // double x = shooter.getXToTarget();
    // System.out.println("x: " + x);
    // if(topShooterSpeed > 4) {
    //   topShooterSpeed = 20;
    // }
    // else {
    //   topShooterSpeed = -1182.5523*Math.pow(x, 3) + 13205.1580*Math.pow(x, 2) - 49074.4837*x + 60713.2697;
    // }
    // topShooterSpeed = 15;
    // bottomShooterSpeed = 90;
    //withTimeout(20);
    // System.out.println("top: " + topShooterSpeed);
    // System.out.println("bottom: " + bottomShooterSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // intake.setValue(-1);
    // ctr += 20;
    // System.out.println("ctr: " + ctr);
    // if (shooter.hasValidTargetPose3d())
    // {
      if(MathUtil.withinTolerance(shooter.getTopVelocity(), topShooterSpeed, 3)) {
        shooter.servoOpen();
      }
      shooter.tpid.setSetpoint(topShooterSpeed);
      shooter.bpid.setSetpoint(bottomShooterSpeed);
      double calctop = shooter.tpid.calculate(shooter.getTopVelocity());
      double calcBot = shooter.bpid.calculate(shooter.getBottomVelocity());
      shooter.setTopMotorVoltage(calctop + shooter.tff.calculate(topShooterSpeed));
      shooter.setBottomMotorVoltage(calcBot + shooter.bff.calculate(bottomShooterSpeed));
    // }
    // else
    // {
    //   shooter.stop();
    // }
    // if(ctr >= timeout) {
    //   end(false);
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.servoClose();
    shooter.stop();
    leds.turnOff();
    // intake.setValue(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (!shooter.hasValidTargetPose3d())
    {
      return true;
    }
    else
    {
      return false;
    }

    // System.out.println(ctr >= timeout);
    // return ctr >= timeout;
    
  }
}
