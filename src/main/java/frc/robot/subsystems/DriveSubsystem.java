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
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.kauailabs.navx.frc.AHRS;

import static frc.robot.Constants.*;

import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

public class DriveSubsystem extends SubsystemBase implements Loggable {
  private final CANSparkMax _left1 = new CANSparkMax(DriveConstants.CAN_ID_LEFT_DRIVE,MotorType.kBrushless);
  private final CANSparkMax _right1 = new CANSparkMax(DriveConstants.CAN_ID_RIGHT_DRIVE,MotorType.kBrushless);
  private final CANSparkMax _left2 = new CANSparkMax(DriveConstants.CAN_ID_LEFT_DRIVE_2,MotorType.kBrushless);
  private final CANSparkMax _right2 = new CANSparkMax(DriveConstants.CAN_ID_RIGHT_DRIVE_2,MotorType.kBrushless);
  
  @Log.SpeedController
  private final SpeedControllerGroup m_right = new SpeedControllerGroup(_right1, _right2);
  @Log.SpeedController
  private final SpeedControllerGroup m_left = new SpeedControllerGroup(_left1, _left2);

  private final DifferentialDrive m_drive = new DifferentialDrive(m_left,  m_right);
  
  public double slewSpeed = 3;  // in units/s
  public double slewTurn = 5;
  private final SlewRateLimiter m_speedSlew = new SlewRateLimiter(slewSpeed);
  private final SlewRateLimiter m_turnSlew = new SlewRateLimiter(slewTurn);

  // setup ultrasonic sensor
  private final AnalogInput m_ultrasonic = new AnalogInput(DriveConstants.kUltrasonicPort);

  private final CANEncoder m_encoderRight;
  private final CANEncoder m_encoderLeft;
  private final DifferentialDriveOdometry m_odometry;
  private final Gyro m_gyro = new AHRS(SPI.Port.kMXP);

 
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

    _left1.burnFlash();
    _left2.burnFlash();
    _right1.burnFlash();
    _right2.burnFlash();

    // Set drive deadband and safety 
    m_drive.setDeadband(0.05);
    m_drive.setSafetyEnabled(true);

    m_encoderLeft = _left1.getEncoder();
    m_encoderRight = _right1.getEncoder();
    m_encoderLeft.setPositionConversionFactor(DriveConstants.kEncoderDistanceConversionFactor);
    m_encoderRight.setPositionConversionFactor(DriveConstants.kEncoderDistanceConversionFactor);
    m_encoderLeft.setVelocityConversionFactor(DriveConstants.kEncoderVelocityConversionFactor);
    m_encoderRight.setVelocityConversionFactor(DriveConstants.kEncoderVelocityConversionFactor);
    resetEncoders();

    zeroHeading();
    

    

    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
    
  }

  public void periodic(){
    m_odometry.update(Rotation2d.fromDegrees(getHeading()), m_encoderLeft.getPosition(), 
    m_encoderRight.getPosition());
    
    SmartDashboard.putNumber("Heading", getHeading());
    SmartDashboard.putNumber("Angle", m_gyro.getAngle());
    SmartDashboard.putNumber("Pos Left", m_encoderLeft.getPosition());
    SmartDashboard.putNumber("Pos Right", m_encoderRight.getPosition());
    SmartDashboard.putNumber("Vel Left", m_encoderLeft.getVelocity());
    SmartDashboard.putNumber("Vel Right", m_encoderRight.getVelocity());
  }

  public Pose2d getPose() {
      return m_odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
      return new DifferentialDriveWheelSpeeds(m_encoderLeft.getVelocity(),
      m_encoderRight.getVelocity()); 
  }

  public void resetOdometry() {
      resetEncoders();

      //m_gyro.reset(); //Check
      m_odometry.resetPosition(new Pose2d(0,0, new Rotation2d(0)), Rotation2d.fromDegrees(getHeading()));
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
      m_left.setVoltage(leftVolts);
      m_right.setVoltage(-rightVolts);
      m_drive.feed();
    }

  public double getHeading() {
      return Math.IEEEremainder(m_gyro.getAngle(), 360) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);//gyro is inversed
  }

  /**
   * Zeroes the heading of the robot.
   */
  public void zeroHeading() {
    m_gyro.reset();
  }


  public double getTurnRate() {
      return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  public void resetEncoders() {
    m_encoderRight.setPosition(0);
    m_encoderLeft.setPosition(0);
    m_gyro.reset();
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    double speedLimit = 0.9;
    m_drive.arcadeDrive(speedLimit*m_speedSlew.calculate(-fwd), m_turnSlew.calculate(rot));
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

  
   /**
   * Sets the max output of the drive.  Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  @Config(name="Max Drive Output", defaultValueNumeric = 1)
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  @Log(name = "Sonar Distance Inches")
  public double getSonarDistanceInches(){
    return m_ultrasonic.getValue()*DriveConstants.kValueToInches;
  }
  
  public void invertRightMotors(){  //this is a temporary fix 
    _left1.setInverted(false);
    _left2.setInverted(false);
    _right1.setInverted(true);   
    _right2.setInverted(true); // invalid pid from characterization tool
}
  public void stop(){
    _left1.stopMotor();
    _left2.stopMotor();
    _right1.stopMotor();
    _right2.stopMotor();
  }  

  public void setSafety(){
    m_drive.setSafetyEnabled(true);
}
public void setNoSafety(){
    m_drive.setSafetyEnabled(false);  
}
}