/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pneumatics extends SubsystemBase {
  
  Compressor compressor = new Compressor(0);

  DoubleSolenoid sol = new DoubleSolenoid(1, 0);
  
  /**
   * Creates a new Pneumatics.
   */
  public Pneumatics() {
    compressor.start();
    TurnOffSolenoid();
    CloseSolenoid();
  }

  public void OpenSolenoid()
  {
    sol.set(Value.kForward);
  }

  public void CloseSolenoid()
  {
    sol.set(Value.kReverse);
  }

  public void TurnOffSolenoid()
  {
    sol.set(Value.kOff);
  }

  public void TurnOffCompressor()
  {
    compressor.stop();
  }

  public void TurnOnCompressor()
  {
    compressor.start();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
