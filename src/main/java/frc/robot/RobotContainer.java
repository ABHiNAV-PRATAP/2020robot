/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.auto.SixBall;
import frc.robot.commands.drivetrain.ArcadeDrive;
import frc.robot.commands.drivetrain.CuravtureDrive;
import frc.robot.commands.drivetrain.FlipDrivetrain;
import frc.robot.commands.drivetrain.PIDRotateAngle;
import frc.robot.commands.intake.IntakeCell;
import frc.robot.commands.intake.OuttakeCell;
import frc.robot.commands.shooter.Shoot;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LimelightVision;
import frc.robot.subsystems.Pneumatics;
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

  private final LimelightVision limelight = new LimelightVision();

  private final Pneumatics pneumatics = new Pneumatics();

  private Joystick driveJoystick = new Joystick(Constants.kDriveJoystickPort);

  private JoystickButton shoot9020 = new JoystickButton(driveJoystick, 12);
  private JoystickButton shoot9015 = new JoystickButton(driveJoystick, 2);
  private JoystickButton shoot9010 = new JoystickButton(driveJoystick, 10);

  private JoystickButton servoTest = new JoystickButton(driveJoystick, 7);


  private final JoystickButton align = new JoystickButton(driveJoystick, 8);

  // private final JoystickButton outreach = new JoystickButton(driveJoystick, 11);
  
 private final JoystickButton flipDT = new JoystickButton(driveJoystick, 6);
  
  private final JoystickButton shoot = new JoystickButton(driveJoystick, 1);
  //private final JoystickButton rotate = new JoystickButton(driveJoystick, 10);

  SendableChooser<CommandBase> autonomousTrajectories;

  private final JoystickButton intakeCell = new JoystickButton(driveJoystick, 3);
  private final JoystickButton outtakeCell = new JoystickButton(driveJoystick, 4);
  
  private final VisionLEDs leds = new VisionLEDs();

  private final JoystickButton compressorOff = new JoystickButton(driveJoystick, 11);
  private final JoystickButton compressorOn = new JoystickButton(driveJoystick, 9);

  //public final DoubleSolenoid sol = new DoubleSolenoid(0, 1);

  // private final XboxController controller = new XboxController(1);

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();



    autonomousTrajectories = new SendableChooser<>();

    // drivetrain.setDefaultCommand(
    //   new TankDrive(
    //     drivetrain, 
    //     () -> -controller.getY(Hand.kLeft), 
    //     () -> -controller.getY(Hand.kRight)
    //   )
    // );
    // drivetrain.setDefaultCommand(
    //   new CuravtureDrive(drivetrain, () -> -driveJoystick.getY(), () ->driveJoystick.getX(), () -> false, () -> driveJoystick.getThrottle()));

    drivetrain.setDefaultCommand(new ArcadeDrive(drivetrain, () -> -driveJoystick.getY(), () -> driveJoystick.getX(), () -> driveJoystick.getThrottle()));
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

    // new JoystickButton(driveJoystick, 11).whenPressed(new RunCommand(() -> shooter.setServoAngle(60)));

    align.whileHeld(
      new PIDRotateAngle(
        drivetrain,
        shooter,
        leds
      )
    );

    // shoot.whileHeld(new Shoot(shooter, () -> shooter.topSetpointShuffleboard.getDouble(0), 
    // () -> shooter.bottomSetpointShuffleboard.getDouble(0), leds));
    
    // shoot.whileHeld(new PIDRotateAngle(drivetrain, shooter, leds).andThen(new TeleopVisionAssistedShoot(shooter)));
    shoot9015.whileHeld(new Shoot(shooter, () -> 11.5, () -> 90, leds, pneumatics));
    shoot9020.whileHeld(new Shoot(shooter, () -> 20, () -> 90, leds, pneumatics));
    // shoot9010.whileHeld(new Shoot(shooter, () -> 10, () -> 90, leds, pneumatics));
    shoot9010.whenPressed(new InstantCommand(pneumatics::TurnOffSolenoid, pneumatics));
    shoot.whileHeld(new Shoot(shooter, () -> 13, () -> 90, leds, pneumatics));

    flipDT.whenPressed(new FlipDrivetrain(drivetrain));
    
    compressorOff.whenPressed(new InstantCommand(pneumatics::TurnOffCompressor, pneumatics));
    compressorOn.whenPressed(new InstantCommand(pneumatics::TurnOnCompressor, pneumatics));

    servoTest.whileHeld(new Shoot(shooter, () -> 0, () -> 0, leds, pneumatics));
    // rotate.whenPressed(new TurnToAngle(drivetrain, 180));
    // rotate.whenPressed(new UpdateTargetPose(shooter, leds).andThen(new PIDRotateAngle(drivetrain, shooter, leds)));
    // rotate.whileHeld(
    //   new PIDRotateAngle(drivetrain, shooter, leds).andThen(
    //     new Shoot(
    //       shooter,
    //       () -> shooter.topSetpointShuffleboard.getDouble(0.0),
    //       () -> shooter.bottomSetpointShuffleboard.getDouble(0.0),
    //       leds
    //     )
    //   )
    // );
    // rotate.whenPressed(new UpdateTargetPose(shooter, leds));
    //rotate.whenPressed(new DriveDistanceStraight(drivetrain, 175)); //setpoint in inches

   //\ flipDT.whenPressed(new FlipDrivetrain(drivetrain));

    intakeCell.whileHeld(new IntakeCell(intake, shooter, pneumatics));
    outtakeCell.whileHeld(new OuttakeCell(intake));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new SixBall(drivetrain, intake, shooter, leds);
  }
}
