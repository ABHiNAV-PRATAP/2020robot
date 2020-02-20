/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.util.MathUtil;

public class PIDRotateAngle extends CommandBase {

  // private double[] samples;
  // private int i;

  private final Drivetrain drivetrain;
  private final double setpoint;

  private final double kP = 0.005;
  private final double kI = 0.005;
  private final double kD = 0.0001;

  private double previousError = 0;
  private double error = 0;
  private double accumulatedError = 0;
  /**
   * Creates a new PIDRotateAngle.
   */
  public PIDRotateAngle(Drivetrain drivetrain, double setpoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.setpoint = setpoint;
    // this.samples = new double[50];
    // i = 0;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    error = setpoint - drivetrain.getHeadingAsAngle();
    accumulatedError+=error * Constants.kDT;
    // samples[i % 50] = error;
    double output = (kP * error) + (kD * (error - previousError) / Constants.kDT) + (kI * accumulatedError);
    output = -MathUtil.constrain(-0.3, 0.3, output);
    drivetrain.arcadeDrive(0, output);
    previousError = error;
    System.out.println(drivetrain.getHeadingAsAngle());
    // i++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return MathUtil.withinTolerance(getSampleAverage(), setpoint, 5); // Within 5 degrees
    // return MathUtil.withinTolerance(drivetrain.getHeadingAsAngle(), setpoint, 1);
    return false;
  }

  // public double getSampleAverage() {
  //   double sum = 0;
  //   for(int i=0; i<50; i++) {
  //     sum += samples[i];
  //   }
  //   return sum/50;
  // }
}
