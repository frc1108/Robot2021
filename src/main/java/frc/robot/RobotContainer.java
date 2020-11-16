/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import io.github.oblarg.oblog.annotations.Log;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.BallLauncher;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.LightsSubsystem;

import frc.robot.commands.hopper.ManualHopper;
import frc.robot.commands.light.SetSolidColor;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.PWMConstants;
import frc.robot.commands.auto.BasicCommandGroup;
import frc.robot.commands.auto.SimpleAutoGroup;
import frc.robot.commands.drive.FieldOrientedTurn;
import frc.robot.commands.auto.Center8BallAuto;

import static frc.robot.Constants.OIConstants.*;

import java.util.List;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  @Log private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  @Log private final HopperSubsystem m_hopper = new HopperSubsystem();
  @Log private final BallLauncher m_launcher = new BallLauncher();
  private final ClimberSubsystem m_climber = new ClimberSubsystem();
  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  private final FeederSubsystem m_feeder = new FeederSubsystem();
  private final LightsSubsystem m_lights = new LightsSubsystem(PWMConstants.PWM_ID_LEDS,160);

  // A chooser for autonomous commands
  SendableChooser<Command> m_chooser = new SendableChooser<>();
  SendableChooser<Command> m_ledChooser = new SendableChooser<>();

  // Controller for driver and operator
  XboxController m_driverController = new XboxController(kDriverControllerPort);
  XboxController m_operatorController = new XboxController(kOperatorControllerPort);

  SneakyTrajectory s_trajectory = new SneakyTrajectory(m_robotDrive);

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    configureButtonBindings();

    // Drive default is split stick arcade drive (fwd left / rot right)
    m_robotDrive.setDefaultCommand(
        new RunCommand(()->m_robotDrive.arcadeDrive(m_driverController.getY(GenericHID.Hand.kLeft),
            m_driverController.getX(GenericHID.Hand.kRight)), m_robotDrive));

    // Hopper axle default
    m_hopper.setDefaultCommand(
        new ManualHopper(m_hopper, () -> m_operatorController.getY(GenericHID.Hand.kRight)));
      
    // Winch default 
    m_climber.setDefaultCommand(
        new RunCommand(()->m_climber.manualControl(m_operatorController.getY(GenericHID.Hand.kLeft)),
            m_climber));

    // Add commands to the autonomous command chooser
    m_chooser.setDefaultOption("Drive Off Line Auto", new SimpleAutoGroup(m_robotDrive));
    m_chooser.addOption("Regular Auto", new BasicCommandGroup(m_robotDrive, m_launcher, m_feeder, m_hopper));
    
    m_ledChooser.setDefaultOption("None", new RunCommand(() -> m_lights.setSolidColor(0, 0, 0)));
    m_ledChooser.addOption("Red", new SetSolidColor(m_lights,255,0,0));
    m_ledChooser.addOption("Blue", new SetSolidColor(m_lights,0,0,255));
    
    Shuffleboard.getTab("Setup").add(m_chooser);
    Shuffleboard.getTab("Setup").add(m_ledChooser);
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Reverse feeder motor for time with operator button Y
    new JoystickButton(m_operatorController, XboxController.Button.kY.value)
        .toggleWhenActive(new StartEndCommand(
                              ()->m_feeder.slowOutFeeder(),
                              ()->m_feeder.stop(),m_feeder)
                              .withTimeout(0.2));

   // Run feeder motor for time with operator button B
    new JoystickButton(m_operatorController, XboxController.Button.kB.value)
        .toggleWhenActive(new WaitCommand(0.8)
                              .andThen(new StartEndCommand(
                                           ()->m_feeder.fastInFeeder(),
                                           ()->m_feeder.stop(),m_feeder)
                                           .withTimeout(5)));

   
    // Run feeder motor for time with operator button X
    new JoystickButton(m_operatorController, XboxController.Button.kX.value)
        .toggleWhenActive(new StartEndCommand(
                              ()->m_feeder.slowInFeeder(),
                              ()->m_feeder.stop(),m_feeder)
                              .withTimeout(0.6));

    new JoystickButton(m_operatorController, XboxController.Button.kBumperRight.value)
      .whileHeld(new RunCommand(() -> m_intake.startIntake(), m_intake));

    new JoystickButton(m_operatorController, XboxController.Button.kBumperLeft.value)
      .whileHeld(new RunCommand(() -> m_intake.slowOutIntake(), m_intake));
    
    new POVButton(m_operatorController, 180)  // Xbox down arrow
      .whenPressed(new RunCommand(() -> m_hopper.down(), m_hopper).withTimeout(2)
      .withInterrupt(m_hopper::isLowSwitchSet));

    new POVButton(m_operatorController, 0)  // Xbox up arrow
      .whenPressed(new RunCommand(() -> m_hopper.up(), m_hopper).withTimeout(2)
      .withInterrupt(m_hopper::isHighSwitchSet));

    new JoystickButton(m_operatorController, XboxController.Button.kB.value)
      .whenPressed(new RunCommand(()-> m_launcher.startPIDLauncher(), m_launcher).withTimeout(6));
     
    new JoystickButton(m_operatorController, XboxController.Button.kStart.value)
      .whenPressed(new RunCommand(()-> m_climber.setSpeedMax(),m_climber).withTimeout(0.1));

    new JoystickButton(m_driverController, XboxController.Button.kB.value)
      .whenPressed(new FieldOrientedTurn(120,m_robotDrive));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
 /*    public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  } */
  
  public Command getAutoCommand() {
    return new Center8BallAuto(s_trajectory,m_robotDrive);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
      // Create a voltage constraint to ensure we don't accelerate too fast
      var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
          new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter,
              DriveConstants.kaVoltSecondsSquaredPerMeter),
              DriveConstants.kDriveKinematics, 10);

      // Create config for trajectory
      TrajectoryConfig config = new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond,
          AutoConstants.kMaxAccelerationMetersPerSecondSquared)
              // Add kinematics to ensure max speed is actually obeyed
              .setKinematics(DriveConstants.kDriveKinematics)
              // Apply the voltage constraint
              .addConstraint(autoVoltageConstraint)
              .setReversed(false);

      // An example trajectory to follow. All units in meters.
      Trajectory driveToGoal = TrajectoryGenerator.generateTrajectory(
          // Start at the origin facing the +X direction
          new Pose2d(0,-2, new Rotation2d(Units.degreesToRadians(0))),
          // Pass through these two interior waypoints, making an 's' curve path
          List.of(),
          // End 3 meters straight ahead of where we started, facing forward
          new Pose2d(2.5,-2.5, new Rotation2d(Units.degreesToRadians(-45))),
          // Pass config
          config);
      
      m_robotDrive.resetOdometry(driveToGoal.getInitialPose());

      // Paste this variable in
      RamseteController disabledRamsete = new RamseteController() {
      @Override
      public ChassisSpeeds calculate(Pose2d currentPose, Pose2d poseRef, double linearVelocityRefMeters,
          double angularVelocityRefRadiansPerSecond) {
      return new ChassisSpeeds(linearVelocityRefMeters, 0.0, angularVelocityRefRadiansPerSecond);
          }
      };

      var table = NetworkTableInstance.getDefault().getTable("troubleshooting");
      var leftReference = table.getEntry("left_reference");
      var leftMeasurement = table.getEntry("left_measurement");
      var rightReference = table.getEntry("right_reference");
      var rightMeasurement = table.getEntry("right_measurement");
      var leftController = new PIDController(DriveConstants.kPDriveVel, 0, 0);
      var rightController = new PIDController(DriveConstants.kPDriveVel, 0, 0);

      RamseteCommand ramseteCommand = new RamseteCommand(driveToGoal, m_robotDrive::getPose,
          //disabledRamsete,
          new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
          new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter,
              DriveConstants.kaVoltSecondsSquaredPerMeter),
          DriveConstants.kDriveKinematics, 
          m_robotDrive::getWheelSpeeds,
          leftController,
          rightController,
          //new PIDController(DriveConstants.kPDriveVel, 0, 0),  // Left PID
          //new PIDController(DriveConstants.kPDriveVel, 0, 0), // Right PID
          // RamseteCommand passes volts to the callback
          // RamseteCommand passes volts to the callback
          (leftVolts, rightVolts) -> {
              m_robotDrive.tankDriveVolts(leftVolts, rightVolts);

              leftMeasurement.setNumber(m_robotDrive.getWheelSpeeds().leftMetersPerSecond);
              leftReference.setNumber(leftController.getSetpoint());

              rightMeasurement.setNumber(m_robotDrive.getWheelSpeeds().rightMetersPerSecond);
              rightReference.setNumber(rightController.getSetpoint());
          },
          //m_robotDrive::tankDriveVolts,
          m_robotDrive);

      // Run path following command, then stop at the end.
      return ramseteCommand.andThen(() -> m_robotDrive.tankDriveVolts(0, 0));
      //return m_traj.getRamsete(m_traj.rightAuto8Cell[0]).andThen(() -> m_robotDrive.tankDriveVolts(0, 0));
  }

   public Command getLightInitCommand() {
    return m_ledChooser.getSelected();
  } 

  public void reset(){
    m_robotDrive.reset();
  }
}
