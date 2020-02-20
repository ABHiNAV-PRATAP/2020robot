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

public class UpdateTargetPose extends CommandBase {
  private final Shooter shooter;
  private final VisionLEDs leds;

  private boolean foundValidTarget2d;
  private boolean foundValidTarget3d;

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
    leds.turnOn();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!foundValidTarget2d)
      foundValidTarget2d = shooter.setTargetPose2d(shooter.getTargetPose2d());
    if(!foundValidTarget3d)
      foundValidTarget3d = shooter.setTargetPose3d(shooter.getTargetPose3d());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    leds.turnOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return foundValidTarget2d && foundValidTarget3d;
  }
}
