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
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.VisionLEDs;
import frc.util.MathUtil;

public class PIDRotateAngle extends CommandBase {

  private final Drivetrain drivetrain;
  private final Shooter shooter;
  private final VisionLEDs leds;
  private double setpoint;

  private final double kP = 0.02; // 0.06//0.0825;//0.075;// 0.0875; // 0.09; //0.1; // 0.1125;//0.125; //0.15;
  private final double kD = 0.0095; //0.011;// 0.009875;// 0.00975;
  // Use 0.0825 & 0.00975 for drivetrain rotation

  private double previousError = 0;
  private double error = 0;

  private int stopAccumulator;

  /**
   * Creates a new PIDRotateAngle.
   */
  public PIDRotateAngle(Drivetrain drivetrain, Shooter shooter, VisionLEDs leds) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.shooter = shooter;
    this.leds = leds;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Started Rotate");
    stopAccumulator = 0;
    setpoint = 0;
    leds.turnOn();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.setTargetPose2d(shooter.getTargetPose2d());
    shooter.setTargetPose3d(shooter.getTargetPose3d());
    error = -shooter.getYawToTarget();
    double output = (kP * error) + (kD * (error - previousError) / Constants.kDT);
    drivetrain.arcadeDrive(0, -output);
    previousError = error;
    System.out.println(drivetrain.getHeadingAsAngle());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Math.abs(error)<=1){
      stopAccumulator++;
    }
    else if(Math.abs(error)>=1){
      stopAccumulator = 0;
    }
    return stopAccumulator>=10;
  }
}
