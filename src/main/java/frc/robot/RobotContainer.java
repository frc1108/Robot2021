/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.BallLauncher;
import frc.robot.subsystems.UsbSerial;

import frc.robot.commands.DefaultLauncher;
import frc.robot.commands.DefaultIntake;
import frc.robot.commands.RunIntake;
import frc.robot.commands.LowerWhopper;
import frc.robot.commands.RaiseHopper;
import frc.robot.commands.ManualHopper;
import frc.robot.commands.ReadGyro;

import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

import static frc.robot.Constants.OIConstants.kDriverControllerPort;
import static frc.robot.Constants.IntakeConstants.hopperIntakeSpeed;
import static frc.robot.Constants.IntakeConstants.launcherIntakeSpeed;


/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems
  @Config.NumberSlider(name = "Max Drive Output",
                       methodName = "setMaxOutput",
                       methodTypes = {double.class},
                       defaultValue = 1)
  @Log
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  
  @Log
  private final BallLauncher m_robotLaunch = new BallLauncher();
  
  @Log
  private final UsbSerial gyro = new UsbSerial();

  private final double ballSpeed = 0.60;

  @Log
  private final IntakeSubsystem m_intakesystem = new IntakeSubsystem();

  @Log
  private final HopperSubsystem m_hoppersystem = new HopperSubsystem();

  // A simple autonomous routine that does something
  @Config.Command(name = "Autonomous Command")
  private final Command m_autoCommand =
     // Start by spinning up launcher
    new WaitCommand(1);

  // A chooser for autonomous commands
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  // The driver's controller
  XboxController m_driverController = new XboxController(kDriverControllerPort);

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    

    //gyro.setDefaultCommand(new ReadGyro(gyro));

    m_hoppersystem.setDefaultCommand(
      new ManualHopper(
        m_hoppersystem,
        () -> m_driverController.getY(GenericHID.Hand.kRight)
      )
    );
    
    /* m_intakesystem.setDefaultCommand(
      new DefaultIntake(m_intakesystem,
      () -> hopperIntakeSpeed,
      () -> launcherIntakeSpeed)); */
    
   
      m_robotLaunch.setDefaultCommand(
      
      new DefaultLauncher(
        m_robotLaunch,
        () -> ballSpeed,
        () -> ballSpeed
      )
    ); 


    // Configure default commands
    // Default robot Drive is single-stick curvature drive
    m_robotDrive.setDefaultCommand(
        // A split-stick curvature command, with forward/backward controlled by the left Y
        // hand, and turning controlled by the left X axis, and quick turn on right hand bumper.
        new RunCommand(()->m_robotDrive
            .curvatureDrive(m_driverController.getY(GenericHID.Hand.kLeft),
                            m_driverController.getX(GenericHID.Hand.kLeft),
                            m_driverController.getBumper(GenericHID.Hand.kRight)), m_robotDrive));
    
    // Default robot Launcher is off
    m_robotLaunch.setDefaultCommand(
      new DefaultLauncher(
        m_robotLaunch,
        () -> ballSpeed,
        () -> ballSpeed));

    // Put the chooser on the dashboard
    Shuffleboard.getTab("Autonomous").add(m_chooser);
    Shuffleboard.getTab("Testing").add(m_hoppersystem);
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Grab the hatch when the 'A' button is pressed.
    //new JoystickButton(m_driverController, 1)
    //    .whenPressed(new LowerWhopper(m_hoppersystem));
    // Release the hatch when the 'B' button is pressed.
    //new JoystickButton(m_driverController, 2)
    //    .whenPressed(new RaiseHopper(m_hoppersystem));
    // While holding the shoulder button, drive at half speed
    //new JoystickButton(m_driverController, Button.kBumperRight.value)
    //    .whenHeld(new HalveDriveSpeed(m_robotDrive));
    JoystickButton IntakeButton = new JoystickButton(m_driverController, XboxController.Button.kA.value); 
    IntakeButton.toggleWhenPressed(new RunIntake(m_intakesystem,() -> launcherIntakeSpeed));
    JoystickButton RollerButton = new JoystickButton(m_driverController, XboxController.Button.kBumperRight.value);
    RollerButton.whenPressed(new DefaultIntake(m_intakesystem,() -> hopperIntakeSpeed));
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }
}
