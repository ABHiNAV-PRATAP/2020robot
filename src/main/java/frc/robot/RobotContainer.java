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
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.drivetrain.ArcadeDrive;
import frc.robot.commands.drivetrain.BangBangRotate;
import frc.robot.commands.drivetrain.ExampleCommand;
import frc.robot.commands.drivetrain.PIDRotateAngle;
import frc.robot.commands.drivetrain.RotateToAnglePID;
import frc.robot.commands.drivetrain.TurnToAngle;
import frc.robot.commands.intake.IntakeCell;
import frc.robot.commands.intake.OuttakeCell;
import frc.robot.commands.shooter.Shoot;
import frc.robot.commands.shooter.UpdateTargetPose;
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

  private final JoystickButton align = new JoystickButton(driveJoystick, 8);
  
  private final JoystickButton flipDT = new JoystickButton(driveJoystick, 2);
  
  private final JoystickButton shoot = new JoystickButton(driveJoystick, 1);
  private final JoystickButton rotate = new JoystickButton(driveJoystick, 10);

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
        () -> -driveJoystick.getY(),
        () -> driveJoystick.getX(),
        () -> driveJoystick.getThrottle()
      )
    );
    
    // leds.setDefaultCommand(
    //   new UpdateTargetPose(
    //     shooter, 
    //     leds)
    // );

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

    new JoystickButton(driveJoystick, 9).whenPressed(new RunCommand(() -> shooter.setServoAngle(60)));

    align.whileHeld(
      new PIDRotateAngle(
        drivetrain,
        shooter,
        leds
      )
    );

    shoot.whileHeld(new Shoot(shooter, () -> shooter.topSetpointShuffleboard.getDouble(0), () -> shooter.bottomSetpointShuffleboard.getDouble(0), leds));
    
    rotate.whenPressed(new TurnToAngle(drivetrain, 30));

    flipDT.whenPressed(new RunCommand(() -> drivetrain.flipDT(), drivetrain));

    intakeCell.whileHeld(new IntakeCell(intake));
    outtakeCell.whileHeld(new OuttakeCell(intake));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new ExampleCommand();
  }
}
