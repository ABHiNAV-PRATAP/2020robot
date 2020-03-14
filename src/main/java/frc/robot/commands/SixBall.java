/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.Shooter;

import frc.robot.commands.drivetrain.DriveDistanceStraight;
import frc.robot.commands.drivetrain.PIDRotateAngle;
import frc.robot.commands.drivetrain.TurnToAngle;
import frc.robot.commands.shooter.Shoot;
import frc.robot.commands.shooter.VisionAssistedShoot;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class SixBall extends SequentialCommandGroup {
  /**
   * Creates a new Threeball.
   */

  Drivetrain drivetrain;
  Shooter shooter;
  Pneumatics pneumatics;
  Intake intake;

  public SixBall(Drivetrain drivetrain, Shooter shooter, Pneumatics pneumatics, Intake intake) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    
    super();
    this.drivetrain = drivetrain;
    this.shooter = shooter;
    this.pneumatics = pneumatics;
    this.intake = intake;

    addCommands(
      new InstantCommand(drivetrain::resetGyro, drivetrain),
      new DriveDistanceStraight(drivetrain, intake, .75, 85),
      new PIDRotateAngle(drivetrain, shooter),
      new Shoot(shooter, () -> 11, () -> 90, pneumatics).withTimeout(4.5),
      new InstantCommand(pneumatics::CloseSolenoid, pneumatics),
      new TurnToAngle(drivetrain, 0),
      new DriveDistanceStraight(drivetrain, intake, .75, 75),
      new PIDRotateAngle(drivetrain, shooter),
      new Shoot(shooter, () -> 14, () -> 90, pneumatics).withTimeout(4.5),
      new InstantCommand(pneumatics::CloseSolenoid, pneumatics),
      new TurnToAngle(drivetrain, 0)
    );
  }
}
