/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Drivetrain;
import frc.util.MathUtil;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class RotateToAnglePID extends PIDCommand {
  /**
   * Creates a new RotateToAnglePID.
   */

    Drivetrain dt;


  public RotateToAnglePID(Drivetrain dt, double setpoint) {
    super(
        // The controller that the command will use
        new PIDController(0.01, 0.01, 0.002),
        // This should return the measurement
        () -> -dt.getHeadingAsAngle(),
        // This should return the setpoint (can also be a constant)
        () -> 30,
        // This uses the output
        output -> { 
          System.out.println(dt.getHeadingAsAngle());
          dt.arcadeDrive(0, MathUtil.constrain(-0.3, 0.3, output));
          // Use the output here
        });
        addRequirements(dt);
        this.dt = dt;
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(getController().getPositionError()) < 1;
  }
}
