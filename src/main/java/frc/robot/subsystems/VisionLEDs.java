/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.commands.shooter.UpdateTargetPose;

public class VisionLEDs extends SubsystemBase {
  /**
   * Creates a new VisionLEDs.
   */

  Solenoid led1 = new Solenoid(0);
  Solenoid led2 = new Solenoid(1);

  public VisionLEDs() {
    // turnOn();
    turnOff();
  }

  public void turnOn() {
    setLED1(true);
    setLED2(true);
  }

  public void turnOff() {
    setLED1(false);
    setLED2(false);
  }

  public void setLED1(boolean on) {
    led1.set(on);
  }
  
  public void setLED2(boolean on) {
    led2.set(on);
  }

  public boolean getLEDStatus() { 
    return led1.get() || led2.get();    
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
