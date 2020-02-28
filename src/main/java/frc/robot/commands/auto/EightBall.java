/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drivetrain.DriveDistanceStraight;
import frc.robot.commands.drivetrain.PIDRotateAngle;
import frc.robot.commands.drivetrain.RotateToAngle;
import frc.robot.commands.drivetrain.TurnToAngle;
import frc.robot.commands.intake.IntakeCell;
import frc.robot.commands.shooter.VisionAssistedShoot;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.VisionLEDs;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class EightBall extends SequentialCommandGroup {
  /**
   * Creates a new EightBall.
   */
  public EightBall(Drivetrain drivetrain, Intake intake, Shooter shooter, VisionLEDs leds) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super();
    addCommands(new DriveDistanceStraight(drivetrain, intake, 0.5, 85), 
    new PIDRotateAngle(drivetrain, shooter, leds),
    new VisionAssistedShoot(shooter, intake, 4000), 
    new TurnToAngle(drivetrain, 0), 
    new DriveDistanceStraight(drivetrain, intake, 0.5, 80),
    new WaitCommand(1),
    new TurnToAngle(drivetrain, 5),
    new WaitCommand(1),
    new DriveDistanceStraight(drivetrain, intake, 0.2, 15),
    new WaitCommand(1),
    new DriveDistanceStraight(drivetrain, intake, 0.2, -15),
    new WaitCommand(1),
    new TurnToAngle(drivetrain, 0),
    new WaitCommand(1),
    new DriveDistanceStraight(drivetrain, intake, 0.5, -80),
    new PIDRotateAngle(drivetrain, shooter, leds),
    new VisionAssistedShoot(shooter, intake, 4000)
    );
  }
}
