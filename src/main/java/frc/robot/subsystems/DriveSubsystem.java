/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.FRCLogger.FRCLogger;
import frc.robot.pantherlib.Trajectory6391;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.kauailabs.navx.frc.AHRS;

import static frc.robot.Constants.*;

import java.io.IOException;
import java.nio.file.Paths;

import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

public class DriveSubsystem extends SubsystemBase implements Loggable {

  String[] speedRows = { "leftDrivePrimary", "rightDrivePrimary"};
  String[] positionRows = { "leftDrivePrimary", "rightDrivePrimary"};
  String[] drivetrainRows = { "leftVoltage", "rightVoltage"};
  String[] gyro_logRows = { "headingDegrees"};

  FRCLogger speeds = new FRCLogger("speeds.csv", speedRows);
  FRCLogger positions = new FRCLogger("positions.csv", positionRows);
  FRCLogger drivetrain = new FRCLogger("drivetrain.csv", drivetrainRows);
  FRCLogger gyro_log = new FRCLogger("gyro.csv", gyro_logRows);

  private final CANSparkMax _left1 = new CANSparkMax(DriveConstants.CAN_ID_LEFT_DRIVE,MotorType.kBrushless);
  private final CANSparkMax _right1 = new CANSparkMax(DriveConstants.CAN_ID_RIGHT_DRIVE,MotorType.kBrushless);
  private final CANSparkMax _left2 = new CANSparkMax(DriveConstants.CAN_ID_LEFT_DRIVE_2,MotorType.kBrushless);
  private final CANSparkMax _right2 = new CANSparkMax(DriveConstants.CAN_ID_RIGHT_DRIVE_2,MotorType.kBrushless);
  
  private final SpeedControllerGroup m_right = new SpeedControllerGroup(_right1, _right2);
  private final SpeedControllerGroup m_left = new SpeedControllerGroup(_left1, _left2);
  private final DifferentialDrive m_drive = new DifferentialDrive(m_left,  m_right);

  private final PIDController leftPID = new PIDController(DriveConstants.kPDriveVel, 0, 0);
  private final PIDController rightPID = new PIDController(DriveConstants.kPDriveVel, 0, 0);
  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(DriveConstants.ksVolts,DriveConstants.kvVoltSecondsPerMeter,DriveConstants.kaVoltSecondsSquaredPerMeter);

  public double slewSpeed = 6;  // in units/s
  public double slewTurn = 6;
  private final SlewRateLimiter m_speedSlew = new SlewRateLimiter(slewSpeed);
  private final SlewRateLimiter m_turnSlew = new SlewRateLimiter(slewTurn);

  // setup ultrasonic sensor
  private final AnalogInput m_ultrasonic = new AnalogInput(DriveConstants.kUltrasonicPort);

  private final CANEncoder m_encoderRight;
  private final CANEncoder m_encoderLeft;
  private final DifferentialDriveOdometry m_odometry;
  private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);

  /**
   * Creates a new DriveSubsystem.
   */
  public DriveSubsystem() {
    // Stops drive motors
    stop();

    // Restores default CANSparkMax settings
    _left1.restoreFactoryDefaults();
    _left2.restoreFactoryDefaults();
    _right1.restoreFactoryDefaults();
    _right2.restoreFactoryDefaults();

    // Set Idle mode for CANSparkMax (brake)
    _left1.setIdleMode(IdleMode.kBrake);
    _left2.setIdleMode(IdleMode.kBrake);
    _right1.setIdleMode(IdleMode.kBrake);
    _right2.setIdleMode(IdleMode.kBrake);

    // Set Smart Current Limit for CAN SparkMax
    _left1.setSmartCurrentLimit(40, 60);
    _left2.setSmartCurrentLimit(40, 60);
    _right1.setSmartCurrentLimit(40, 60);
    _right2.setSmartCurrentLimit(40, 60);

    m_encoderLeft = _left1.getEncoder();
    m_encoderRight = _right1.getEncoder();
    m_encoderRight.setPositionConversionFactor(DriveConstants.kEncoderDistanceConversionFactor);
    m_encoderRight.setVelocityConversionFactor(DriveConstants.kEncoderVelocityConversionFactor);
    m_encoderLeft.setPositionConversionFactor(DriveConstants.kEncoderDistanceConversionFactor);
    m_encoderLeft.setVelocityConversionFactor(DriveConstants.kEncoderVelocityConversionFactor);

    _left1.burnFlash();
    _left2.burnFlash();
    _right1.burnFlash();
    _right2.burnFlash();

    // Set drive deadband and safety 
    m_drive.setDeadband(0.05);
    m_drive.setSafetyEnabled(false);

    //m_gyro.reset();
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
  }

  public void periodic(){
    m_odometry.update(Rotation2d.fromDegrees(getHeading()), m_encoderLeft.getPosition(), -m_encoderRight.getPosition());
    SmartDashboard.putNumber("Left Dist", m_encoderLeft.getPosition());
    SmartDashboard.putNumber("Right Dist", -m_encoderRight.getPosition());
    SmartDashboard.putNumber("Left Vel Factor", m_gyro.getAngle());

    speeds.csv.LogWithTime(m_encoderLeft.getVelocity() + "," + m_encoderRight.getVelocity());
    positions.csv.LogWithTime(m_encoderLeft.getPosition() + "," + -m_encoderRight.getPosition());
    gyro_log.csv.LogWithTime(m_gyro.getAngle());
  }

  public Pose2d getPose() {
      return m_odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    zeroHeading();
    m_odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
  }

  public void resetEncoders() {
    m_encoderRight.setPosition(0);
    m_encoderLeft.setPosition(0);
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
      return new DifferentialDriveWheelSpeeds(m_encoderLeft.getVelocity(),-m_encoderRight.getVelocity());
  } 
  
  public void tankDriveVolts(double leftVolts, double rightVolts) {
      m_left.setVoltage(leftVolts);
      m_right.setVoltage(-rightVolts);
      m_drive.feed();
  }

  public void tankDriveWithFeedforwardPID(double leftVelocitySetpoint, double rightVelocitySetpoint) {
      m_left.setVoltage(feedforward.calculate(leftVelocitySetpoint)
          + leftPID.calculate(m_encoderLeft.getVelocity(), leftVelocitySetpoint));
      m_right.setVoltage(feedforward.calculate(rightVelocitySetpoint)
          + rightPID.calculate(-m_encoderRight.getVelocity(), rightVelocitySetpoint));
    m_drive.feed();
}

  @Log
  public double getHeading() {
      return Math.IEEEremainder(m_gyro.getAngle(), 360) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);//gyro is inversed
  }

  /**
   * Zeroes the heading of the robot.
   */
  public void zeroHeading() {
    m_gyro.reset();
  }

  public void reset(){
    m_gyro.reset();
    m_encoderLeft.setPosition(0);
    m_encoderRight.setPosition(0);
    m_odometry.resetPosition(new Pose2d(), Rotation2d.fromDegrees(getHeading()));
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(m_speedSlew.calculate(-fwd), m_turnSlew.calculate(rot));
  }

   /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   * @param quickTurn button to quickTurn
   */
  public void curvatureDrive(double fwd, double rot, boolean quickTurn) {
    m_drive.curvatureDrive(m_speedSlew.calculate(-fwd), m_turnSlew.calculate(rot), quickTurn);
  }
  
  @Config(name="Speed, max", defaultValueNumeric = 0.7)
  public void setSpeedMax(double xSpeed) {
    m_drive.setMaxOutput(xSpeed);
  }

  @Log(name = "Ultrasonic, in")
  public double getSonarDistanceInches(){
    return m_ultrasonic.getValue()*DriveConstants.kValueToInches;
  }
  
  public void stop(){
    _left1.stopMotor();
    _left2.stopMotor();
    _right1.stopMotor();
    _right2.stopMotor();
  }  

  @Log
  public double getHeadingCW() {
    // Not negating
    return Math.IEEEremainder(m_gyro.getAngle(), 360);
  }

  public double getTurnRateCW() {
    // Not negating
    return m_gyro.getRate();
  }

  /**
   * Creates a command to follow a Trajectory on the drivetrain.
   * @param trajectory trajectory to follow
   * @return command that will run the trajectory
   */
  public Command createCommandForTrajectory(Trajectory trajectory, Boolean initPose) {
    if (initPose) {
      new InstantCommand(() -> {resetOdometry(trajectory.getInitialPose());});
    }

    resetEncoders();

    RamseteCommand ramseteCommand =  new RamseteCommand(trajectory, this::getPose,
    new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
    new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter,
                    DriveConstants.kaVoltSecondsSquaredPerMeter),
    DriveConstants.kDriveKinematics, this::getWheelSpeeds,
    new PIDController(DriveConstants.kPDriveVel, 0, 0),
    new PIDController(DriveConstants.kPDriveVel, 0, 0), this::tankDriveVolts, this);
    return ramseteCommand.andThen(() -> this.tankDriveVolts(0, 0));
  }

  protected static Trajectory loadTrajectory(String trajectoryName) throws IOException {
    return TrajectoryUtil.fromPathweaverJson(
        Filesystem.getDeployDirectory().toPath().resolve(Paths.get("paths", trajectoryName + ".wpilib.json")));
  }

  public Trajectory generateTrajectory(String trajectoryName, TrajectoryConfig config) {
    try {
      var filepath = Filesystem.getDeployDirectory().toPath().resolve(Paths.get("waypoints", trajectoryName));
      return Trajectory6391.fromWaypoints(filepath, config);
    } catch (IOException e) {
      DriverStation.reportError("Failed to load auto trajectory: " + trajectoryName, false);
      return new Trajectory();
    }
  }
  public Trajectory loadTrajectoryFromFile(String filename) {
    try {
      return loadTrajectory(filename);
    } catch (IOException e) {
      DriverStation.reportError("Failed to load auto trajectory: " + filename, false);
      return new Trajectory();
    }
  }

  public Trajectory generateTrajectoryFromFile(String filename) {
      var config = new TrajectoryConfig(1, 3);
      return generateTrajectory(filename, config);
  }
}