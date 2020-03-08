/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shooter;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.TalonFXInvertType;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.VisionLEDs;
import frc.util.MathUtil;

public class Shoot extends CommandBase {
  private final Shooter shooter;
  private final DoubleSupplier topSetpoint;
  private final DoubleSupplier bottomSetpoint;
  private final Pneumatics pneumatics;
  private final VisionLEDs leds;
  /**
   * Creates a new Shoot.
   */
  public Shoot(Shooter shooter, DoubleSupplier topSetpoint, DoubleSupplier bottomSetpoint, VisionLEDs leds, Pneumatics pneumatics) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    this.topSetpoint = topSetpoint;
    this.bottomSetpoint = bottomSetpoint;
    this.leds = leds;
    this.pneumatics = pneumatics;
    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    System.out.println("Started Shoot");
    pneumatics.CloseSolenoid();
    //shooter.setServoAngle(60);
    // leds.turnOn();
    // shooter.tpid.setTolerance(1);
    // shooter.bpid.setTolerance(1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      double tsp = topSetpoint.getAsDouble();
      double bsp = bottomSetpoint.getAsDouble();
      double tv = shooter.getTopVelocity();
      double bv = shooter.getBottomVelocity();
      if(MathUtil.withinTolerance(tv, tsp, 3)) {
        shooter.servoOpen();
        pneumatics.OpenSolenoid();
      }
      shooter.tpid.setSetpoint(tsp);
      shooter.bpid.setSetpoint(bsp);
      double calctop = shooter.tpid.calculate(tv);
      double calcBot = shooter.bpid.calculate(bv);
      // System.out.println(calcBot + shooter.bff.calculate(bsp));
      shooter.setTopMotorVoltage(calctop + shooter.tff.calculate(tsp));
      shooter.setBottomMotorVoltage(calcBot + shooter.bff.calculate(bsp));
      // shooter.setTopMotorVoltage(2);
      // shooter.setBottomMotorVoltage(6);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setTopMotorVoltage(0);
    shooter.setBottomMotorVoltage(0);
    shooter.servoClose();
    pneumatics.CloseSolenoid();
    // leds.turnOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
