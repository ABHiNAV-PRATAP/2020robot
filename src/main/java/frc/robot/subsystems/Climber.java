/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PWMTalonSRX;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  /**
   * Creates a new Climber.
   */

  PWMTalonSRX climberWinch = new PWMTalonSRX(1);
  PWMVictorSPX hookDeploy = new PWMVictorSPX(2);

  public Climber() {
    climberWinch.setInverted(true);

  }

  public void setClimberWinchSpeed(double speed){
    climberWinch.set(speed);
  }

  public void setHookDeploySpeed(double speed){
    hookDeploy.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
