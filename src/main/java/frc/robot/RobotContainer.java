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
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.BallLauncher;
import frc.robot.subsystems.UsbSerial;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.LightsSubsystem;
import frc.robot.commands.DefaultLauncher;
import frc.robot.commands.ManualHopper;
import frc.robot.commands.ManualClimber;
import frc.robot.commands.SetSolidColor;
import frc.robot.commands.AutoCommandGroup;

import io.github.oblarg.oblog.annotations.Log;

import static frc.robot.Constants.OIConstants.kDriverControllerPort;
import static frc.robot.Constants.OIConstants.kOperatorControllerPort;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  @Log private final Vision m_vision = new Vision();
  @Log private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  @Log private final IntakeSubsystem m_intakesystem = new IntakeSubsystem();
  @Log private final HopperSubsystem m_hoppersystem = new HopperSubsystem();
  @Log private final FeederSubsystem m_feeder = new FeederSubsystem();
  @Log private final BallLauncher m_robotLaunch = new BallLauncher();
  @Log private final ClimberSubsystem m_climber = new ClimberSubsystem();
  @Log private final LightsSubsystem m_lights = new LightsSubsystem();
  @Log private final UsbSerial gyro = new UsbSerial();
 
  private final Command m_basicAuto = new AutoCommandGroup(m_robotDrive, m_robotLaunch, m_feeder);
  private final Command m_setRedLights = new SetSolidColor(m_lights,255,0,0);
  private final Command m_setBlueLights = new SetSolidColor(m_lights,0,0,255);

 
  // A chooser for autonomous commands
  SendableChooser<Command> m_chooser = new SendableChooser<>();
  SendableChooser<Command> m_ledChooser = new SendableChooser<>();

  // Controller for driver and operator
  XboxController m_driverController = new XboxController(kDriverControllerPort);
  XboxController m_operatorController = new XboxController(kOperatorControllerPort);

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    /**
     * Default command for manual control of subsystem
     */
    // Robot drive default
    m_robotDrive.setDefaultCommand(
        // A split-stick arcade drive with forward/backward controlled by the left Y
        // hand, and turning controlled by the left X axis.
        new RunCommand(()->m_robotDrive
            .arcadeDrive(m_driverController.getY(GenericHID.Hand.kLeft),
                            m_driverController.getX(GenericHID.Hand.kRight)), m_robotDrive));

    // Hopper axle default
    m_hoppersystem.setDefaultCommand(new ManualHopper(
        m_hoppersystem,
        () -> m_operatorController.getY(GenericHID.Hand.kRight)
      )
    );

     // Winch default 
    m_climber.setDefaultCommand(
      new ManualClimber(
        m_climber,
        () -> m_operatorController.getY(GenericHID.Hand.kLeft)
      )
    );

    // Add commands to the autonomous command chooser
    m_chooser.setDefaultOption("Basic Auto", m_basicAuto);
    m_ledChooser.setDefaultOption("Solid Red", m_setRedLights);
    m_ledChooser.addOption("Solid Blue", m_setBlueLights);


    Shuffleboard.getTab("Autonomous").add(m_chooser);
    Shuffleboard.getTab("Lights").add(m_ledChooser);

  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Run feeder motor for time with operator button A
    new JoystickButton(m_operatorController, XboxController.Button.kA.value)
        .toggleWhenActive(new StartEndCommand(
                              ()->m_feeder.slowOutFeeder(),
                              ()->m_feeder.stopFeeder(),m_feeder)
                              .withTimeout(4));

   // Reverse feeder motor for time with operator button B
    new JoystickButton(m_operatorController, XboxController.Button.kB.value)
        .toggleWhenActive(new WaitCommand(0.8)
                              .andThen(new StartEndCommand(
                                           ()->m_feeder.startFeeder(),
                                           ()->m_feeder.stopFeeder(),m_feeder)
                                           .withTimeout(7)));
  
    // Run feeder motor for time with operator button X
    new JoystickButton(m_operatorController, XboxController.Button.kX.value)
        .toggleWhenActive(new StartEndCommand(
                              ()->m_feeder.slowOutFeeder(),
                              ()->m_feeder.stopFeeder(),m_feeder)
                              .withTimeout(0.7));

    // Run intake motor while Operator Right Bumper is pressed
    JoystickButton rollerButton = new JoystickButton(m_operatorController, XboxController.Button.kBumperRight.value);
    rollerButton.whenPressed(new InstantCommand(m_intakesystem::startIntake,m_intakesystem))
                .whenReleased(new InstantCommand(m_intakesystem::stopIntake,m_intakesystem));

    // Reverse intake motor while Operator Left Bumper is pressed                
    JoystickButton outRollerButton = new JoystickButton(m_operatorController, XboxController.Button.kBumperLeft.value);
    outRollerButton.whenPressed(new InstantCommand(m_intakesystem::slowOutIntake,m_intakesystem))
                .whenReleased(new InstantCommand(m_intakesystem::stopIntake,m_intakesystem));
    
    // Run launcher motors when toggling Operator button B.  Simultaneous with feed wheel  
    JoystickButton LaunchButton = new JoystickButton(m_operatorController, XboxController.Button.kB.value);
    LaunchButton.toggleWhenActive(new DefaultLauncher(m_robotLaunch).withTimeout(7));
    
    /* // Servo open close buttons ---- NOT CONNECTED FOR NOW --------------
    JoystickButton ReleaseServoButton = new JoystickButton(m_operatorController, XboxController.Button.kStart.value);
    ReleaseServoButton.whenPressed(new MoveServo(m_climber,() -> servoAngle));
    JoystickButton CloseServoButton = new JoystickButton(m_operatorController, XboxController.Button.kBack.value);
    CloseServoButton.whenPressed(new MoveServo(m_climber,() -> servoCloseAngle)); */
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
    public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }  

   public Command getLightInitCommand() {
    return m_ledChooser.getSelected();
  } 
}
