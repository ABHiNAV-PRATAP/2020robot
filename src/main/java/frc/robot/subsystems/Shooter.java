/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
// neos are spinning backwards so fix tht, spacer is needed 
package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Map;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.util.vision.Interpolator;
import frc.util.vision.ShooterProfile;
import frc.util.vision.VisionTargetPose2d;
import frc.util.vision.VisionTargetPose3d;

public class Shooter extends SubsystemBase {

  private VisionTargetPose2d currentTargetPose2d;
  private VisionTargetPose3d currentTargetPose3d;

  private double x;
  private double yaw;

  private Interpolator interpolator;
  
  double tkP = 0.0075; //0.957;
  final double tkI = 0;
  double tkD = 0;
  final double tkS = 0.0643; //0.0643
  final double tkV = 0.128; //0.128
  final double tkA = 0.0205; //0.0205
  double bkP = 0; // -0.0075; // .01
  final double bkI = 0;
  double bkD = 0;
  final double bkS = 0.0496; // 0.0475; //0.136
  final double bkV = 0.129;// 0.134; //0.128
  final double bkA = 0.0208; // 0.0264; //.0272
  // new characterization
  // forward ks = 0.0523, kv = 0.128, ka = 0.0202, kp = 0.943
  // backward ks = 0.0496, kv = 0.129, ka = 0.0208, kp = 0.972
  final double wheelRadius = Units.inchesToMeters(2);

  CANSparkMax topMotor;
  CANSparkMax bottomMotor;
  Servo servo = new Servo(0);
  public PIDController tpid = new PIDController(tkP, tkI, tkD);
  public SimpleMotorFeedforward tff = new SimpleMotorFeedforward(tkS, tkV, tkA);
  public PIDController bpid = new PIDController(bkP, bkI, bkD);
  public SimpleMotorFeedforward bff = new SimpleMotorFeedforward(bkS, bkV, bkA);
  public ShuffleboardTab tab = Shuffleboard.getTab("Shooter");
  NetworkTableEntry topMotorVoltage = tab.add("Top Motor Voltage", 0).withWidget(BuiltInWidgets.kVoltageView).withProperties(Map.of("Min", -12, "Max", 12, "Center", 0, "Number of Tick Marks", 12)).getEntry();
  NetworkTableEntry bottomMotorVoltage = tab.add("Bottom Motor Voltage", 0).withWidget(BuiltInWidgets.kVoltageView).withProperties(Map.of("Min", -12, "Max", 12, "Center", 0, "Number of Tick Marks", 12)).getEntry();
  NetworkTableEntry topMotorVelocity = tab.add("t vel", 0).withWidget(BuiltInWidgets.kTextView).withSize(2, 1).getEntry();
  NetworkTableEntry bottomMotorVelocity = tab.add("b vel", 0).withWidget(BuiltInWidgets.kTextView).withSize(2, 1).getEntry();
  NetworkTableEntry topMotorError = tab.add("t err", 0).withWidget(BuiltInWidgets.kTextView).withSize(2, 1).getEntry();
  NetworkTableEntry bottomMotorError = tab.add("b err", 0).withWidget(BuiltInWidgets.kTextView).withSize(2, 1).getEntry();
  NetworkTableEntry bottomP = tab.add("bottom P", bkP).withWidget(BuiltInWidgets.kTextView).getEntry();
  NetworkTableEntry topP = tab.add("top P", tkP).withWidget(BuiltInWidgets.kTextView).getEntry();
  NetworkTableEntry bottomD = tab.add("bottom D", bkD).withWidget(BuiltInWidgets.kTextView).getEntry();
  NetworkTableEntry topS = tab.add("top S", tkS).withWidget(BuiltInWidgets.kTextView).getEntry();
  NetworkTableEntry bottomS = tab.add("bottom S", bkS).withWidget(BuiltInWidgets.kTextView).getEntry();
  NetworkTableEntry topV = tab.add("top V", tkV).withWidget(BuiltInWidgets.kTextView).getEntry();
  NetworkTableEntry bottomV = tab.add("bottom V ", bkV).withWidget(BuiltInWidgets.kTextView).getEntry();
  public NetworkTableEntry topSetpointShuffleboard = tab.add("Top Setpoint", 0).withSize(2, 1).withWidget(BuiltInWidgets.kTextView).getEntry();
  public NetworkTableEntry bottomSetpointShuffleboard = tab.add("Bottom Setpoint", 0).withSize(2, 1).withWidget(BuiltInWidgets.kTextView).getEntry();

  NetworkTableEntry xDistanceToTarget = tab.add("x distance", x).withWidget(BuiltInWidgets.kTextView).getEntry();
  NetworkTableEntry yawToTarget = tab.add("yaw", yaw).withWidget(BuiltInWidgets.kTextView).getEntry();

  NetworkTableInstance table = NetworkTableInstance.getDefault();
  NetworkTable cameraTable = table.getTable("chameleon-vision").getSubTable("infuzed-ps3");
  /**
   * Creates a new Shooter.
   */
  public Shooter() {
    topMotor = new CANSparkMax(11, MotorType.kBrushless);
    bottomMotor = new CANSparkMax(10, MotorType.kBrushless);
    topMotor.setInverted(false);
    bottomMotor.setInverted(true);
    topMotor.getEncoder().setPosition(0);
    bottomMotor.getEncoder().setPosition(0);

    servoClose();

    interpolator = new Interpolator(
      // Add and modify these profiles to match the experimental data we collect
      new ArrayList<ShooterProfile>() {{
        // add(new ShooterProfile(x (distance), top shooter speed, bottom shooter speed))
        add(new ShooterProfile(0, 0, 0));
        add(new ShooterProfile(0, 0, 0));
        add(new ShooterProfile(0, 0, 0));
      }}
    );
  }

  public void setTopMotorVoltage(double value) {
    topMotor.setVoltage(value);
  }

  public void servoClose()
  {
    setServoAngle(90);
  }
  public void servoOpen()
  {
    setServoAngle(0);
  }

  public void setServoAngle(double degrees) {
    servo.setAngle(degrees);
  }

  public void setBottomMotorVoltage(double value) {
    bottomMotor.set(value/12);
  }

  public ShooterProfile getShooterProfileFromInterpolator(double distance) {
    return interpolator.interpolate(distance);
  }

  public VisionTargetPose2d getTargetPose2d() {
    if (!cameraTable.getEntry("isValid").getBoolean(false)) {
      // System.out.println("Invalid 2d");
      return null;
    }
    return new VisionTargetPose2d(
      cameraTable.getEntry("targetPitch").getDouble(0.0),
      cameraTable.getEntry("targetYaw").getDouble(0.0),
      cameraTable.getEntry("targetArea").getDouble(0.0)
    );
  }

  public VisionTargetPose3d getTargetPose3d() {
    if (!cameraTable.getEntry("isValid").getBoolean(false)) {
      return null;
    }
    return new VisionTargetPose3d(
      cameraTable.getEntry("targetPose").getDoubleArray(
        new Double[] {0.0, 0.0, 0.0}
      )
    );
  }

  public boolean setTargetPose2d(VisionTargetPose2d targetPose2d) {
    if(targetPose2d == null) {
      return false;
    }
    currentTargetPose2d = targetPose2d;
    return true;
  }

  public boolean setTargetPose3d(VisionTargetPose3d targetPose3d) {
    if(targetPose3d == null) {
      return false;
    }
    currentTargetPose3d = targetPose3d;
    return true;
  }

  public boolean hasValidTargetPose2d() {
    return currentTargetPose2d != null;
  }

  public boolean hasValidTargetPose3d() {
    return currentTargetPose3d != null;
  }

  public void resetTargetPose2d() {
    currentTargetPose2d = null;
  }

  public void resetTargetPose3d() {
    currentTargetPose3d = null;
  }

  public Double getXToTarget() {
    if(currentTargetPose3d == null) {
      return 0.0;
    }
    return currentTargetPose3d.getX();
  }

  public Double getYToTarget() {
    if(currentTargetPose3d == null) {
      return 0.0;
    } 
    return currentTargetPose3d.getY();
  }

  public Double getAngleToTarget() {
    if(currentTargetPose3d == null) {
      return 0.0;
    }
    return currentTargetPose3d.getAngle();
  }

  public Double getPitchToTarget() {
    if(currentTargetPose2d == null) {
      return 0.0;
    }
    return currentTargetPose2d.getPitch();
  }

  public Double getYawToTarget() {
    if(currentTargetPose2d == null) {
      return 0.0;
    }
    return currentTargetPose2d.getYaw();
  }

  public Double getAreaOfTarget() {
    if(currentTargetPose2d == null) {
      return 0.0;
    }
    return currentTargetPose2d.getArea();
  }

  @Override
  public void periodic() {
    x = getXToTarget();
    yaw = getYawToTarget();
  }
  public double getTopVelocity() {
    return topMotor.getEncoder().getVelocity() / 60;
  }
  public double getBottomVelocity() {
    return bottomMotor.getEncoder().getVelocity() / 60;
  }
}