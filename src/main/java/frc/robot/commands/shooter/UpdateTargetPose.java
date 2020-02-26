/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.VisionLEDs;

public class UpdateTargetPose extends CommandBase {
  private final Shooter shooter;
  private final VisionLEDs leds;

  /**
   * Creates a new UpdateTargetPose.
   * Updates both VisionTargetPose2d and VisionTargetPose3d in Shooter subsystem
   */
  public UpdateTargetPose(Shooter shooter, VisionLEDs leds) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    this.leds = leds;
    addRequirements(shooter, leds);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Initializing UpdateTargetPose command");
    leds.turnOn();
    shooter.resetTargetPose2d();
    shooter.resetTargetPose3d();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if(!foundValidTarget2d)
    //   foundValidTarget2d = shooter.setTargetPose2d(shooter.getTargetPose2d());
      // System.out.println(shooter.getTargetPose2d());
    // if(!foundValidTarget3d)
    //   foundValidTarget3d = shooter.setTargetPose3d(shooter.getTargetPose3d());
    // shooter.setTargetPose2d(shooter.getTargetPose2d());
    System.out.println("Execute");
    shooter.setTargetPose2d(shooter.getTargetPose2d());
    shooter.setTargetPose3d(shooter.getTargetPose3d());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // System.out.println("Found valid targets");
    // System.out.println("Current: " + drivetrain.getHeadingAsAngle());
    // System.out.println("Yaw: " + shooter.getYawToTarget());
    // System.out.println("New setpoint: " + (drivetrain.getHeadingAsAngle() - shooter.getYawToTarget()));
    leds.turnOff();
    System.out.println("Ending UpdateTargetPose");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return shooter.hasValidTargetPose2d() && shooter.hasValidTargetPose3d();
    // return false;
  }
}
