/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.drivetrain.ArcadeDrive;
import frc.robot.commands.drivetrain.ExampleCommand;
import frc.robot.commands.drivetrain.PIDRotateAngle;
import frc.robot.commands.drivetrain.RotateToAnglePID;
import frc.robot.commands.intake.IntakeCell;
import frc.robot.commands.intake.OuttakeCell;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.VisionLEDs;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final Drivetrain drivetrain = new Drivetrain();

  private final Intake intake = new Intake();

  private final Shooter shooter = new Shooter();

  private Joystick driveJoystick = new Joystick(Constants.kDriveJoystickPort);

  private final JoystickButton shoot = new JoystickButton(driveJoystick, 8);

  SendableChooser<Trajectory> autonomousTrajectories;

  private final JoystickButton intakeCell = new JoystickButton(driveJoystick, 3);
  private final JoystickButton outtakeCell = new JoystickButton(driveJoystick, 4);

  private final VisionLEDs leds = new VisionLEDs();

  // private final XboxController controller = new XboxController(1);

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();


    // drivetrain.setDefaultCommand(
    //   new TankDrive(
    //     drivetrain, 
    //     () -> -controller.getY(Hand.kLeft), 
    //     () -> -controller.getY(Hand.kRight)
    //   )
    // );
    drivetrain.setDefaultCommand(
      new ArcadeDrive(
        drivetrain,
        () -> driveJoystick.getY(),
        () -> driveJoystick.getX(),
        () -> driveJoystick.getThrottle()
      )
    );
    
    // Shuffleboard.getTab("Auto Commands").add("Auto Mode", autonomousTrajectories);

    // try 
    // {
    //   File folder = new File("/home/lvuser/deploy/");
    //   File[] listOfFiles = folder.listFiles();

    //   for (int i = 0; i < listOfFiles.length; i++) {

    //     if (listOfFiles[i].isFile()) 
    //     {
    //       System.out.println("File " + listOfFiles[i].getName());
    //       autonomousTrajectories.addOption(listOfFiles[i].getName(), TrajectoryUtil.fromPathweaverJson(Paths.get(folder.toString() + listOfFiles[i].getName())));
    //     } 

    //   }
    // }
    // catch (Exception e)
    // {
    //   System.out.println("Unable to populate trajectories!");
    // }
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // shoot.whileHeld(new Shoot(shooter, 
    // () -> shooter.topSetpointShuffleboard.getDouble(0), 
    // () -> shooter.bottomSetpointShuffleboard.getDouble(0),
    // leds));

    // shoot.whileHeld(
    //   new UpdateTargetPose(
    //     shooter,
    //     leds
    //   ).andThen(
    //     new RotateToAngle(
    //       drivetrain, 
    //       // () -> drivetrain.getHeading().getDegrees() + shooter.getYawToTarget()
    //       drivetrain.getHeading().getDegrees() - shooter.getYawToTarget() // Subtract if CCW positive, Add if CW positive
    //     )
    //   ).andThen(
    //     new VisionAssistedShoot(
    //       shooter
    //     )
    //   )
    // );

    shoot.whileHeld(
      new RunCommand(() -> leds.turnOn(), leds).andThen(new RunCommand(() -> leds.turnOff(), leds))
    );

    intakeCell.whileHeld(new IntakeCell(intake));
    outtakeCell.whileHeld(new OuttakeCell(intake));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // var autoVoltageConstraint =
    //     new DifferentialDriveVoltageConstraint(
    //         new SimpleMotorFeedforward(Constants.ksVolts,
    //                                    Constants.kvVoltSecondsPerMeter,
    //                                    Constants.kaVoltSecondsSquaredPerMeter),
    //         Constants.kDriveKinematics,
    //         10);

    // TrajectoryConfig config =
    //     new TrajectoryConfig(Constants.kMaxSpeedMetersPerSecond,
    //                          Constants.kMaxAccelerationMetersPerSecondSquared)           
    //         .setKinematics(Constants.kDriveKinematics)
    //         .addConstraint(autoVoltageConstraint);

    // Trajectory drive3mTrajectory = TrajectoryGenerator.generateTrajectory(
    //     new Pose2d(0, 0, new Rotation2d(0)),
    //     List.of(
    //         new Translation2d(1, 0),
    //         new Translation2d(2, 0)
    //     ),
    //     new Pose2d(3, 0, new Rotation2d(0)),
    //     config
    // );

    // Trajectory traj = null;

    // try 
    // {
    //   traj = autonomousTrajectories.getSelected();
    //   if (traj == null)
    //   {
    //     throw new Exception();
    //   }
    // } 
    // catch (Exception e) 
    // {
    //   System.out.println("Unable to find autonomous file!");
    //   System.out.println("Defaulting to Drive 3 Meters Forward");
    //   traj = drive3mTrajectory;
    // }

    // RamseteCommand ramseteCommand = new RamseteCommand(
    //     traj,
    //     drivetrain::getPose,
    //     new RamseteController(Constants.kRamseteBeta, Constants.kRamseteZeta),
    //     new SimpleMotorFeedforward(Constants.ksVolts,
    //                                Constants.kvVoltSecondsPerMeter,
    //                                Constants.kaVoltSecondsSquaredPerMeter),
    //     Constants.kDriveKinematics,
    //     drivetrain::getWheelSpeeds,
    //     new PIDController(Constants.kPDriveVelocity, 0, 0),
    //     new PIDController(Constants.kPDriveVelocity, 0, 0),
    //     // RamseteCommand passes volts to the callback
    //     drivetrain::setDriveMotorVoltage,
    //     drivetrain
    // );

    // Run path following command, then stop at the end.
    // return ramseteCommand.andThen(() -> drivetrain.setDriveMotorVoltage(0, 0));
    return new ExampleCommand();
  }
}
