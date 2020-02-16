/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.AnalogInput;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.revrobotics.CANEncoder;


import static frc.robot.Constants.DriveConstants.CAN_ID_LEFT_DRIVE;
import static frc.robot.Constants.DriveConstants.CAN_ID_RIGHT_DRIVE;
import static frc.robot.Constants.DriveConstants.CAN_ID_LEFT_DRIVE_2;
import static frc.robot.Constants.DriveConstants.CAN_ID_RIGHT_DRIVE_2;
import static frc.robot.Constants.DriveConstants.kUltrasonicPort;
import static frc.robot.Constants.DriveConstants.kValueToInches;




import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;
import io.github.oblarg.oblog.annotations.Config.Configs;

public class DriveSubsystem extends SubsystemBase implements Loggable {

  
  CANSparkMax _left1 = new CANSparkMax(CAN_ID_LEFT_DRIVE,MotorType.kBrushless);
  CANSparkMax _right1 = new CANSparkMax(CAN_ID_RIGHT_DRIVE,MotorType.kBrushless);
  CANSparkMax _left2 = new CANSparkMax(CAN_ID_LEFT_DRIVE_2,MotorType.kBrushless);
  CANSparkMax _right2 = new CANSparkMax(CAN_ID_RIGHT_DRIVE_2,MotorType.kBrushless);


  @Log.SpeedController(name = "Right Motors")
  SpeedControllerGroup m_right = new SpeedControllerGroup(_right1, _right2);

  @Log.SpeedController(name = "Left Motors")
  SpeedControllerGroup m_left = new SpeedControllerGroup(_left1, _left2);

 
  // The robot's drive
  @Log.DifferentialDrive(name = "Main Drive")
  private final DifferentialDrive m_drive = new DifferentialDrive(m_left,  m_right);
  
  // slew limniter for speed
  public double slewSpeed = 36;
  public double slewTurn = 48;
  private SlewRateLimiter m_speedSlew = new SlewRateLimiter(slewSpeed);
  private SlewRateLimiter m_turnSlew = new SlewRateLimiter(slewTurn);

  // setup ultrasonic sensor
  private final AnalogInput m_ultrasonic = new AnalogInput(kUltrasonicPort);
  

  /**
   * Creates a new DriveSubsystem.
   */
  public DriveSubsystem() {
   // Zeroes drive motor output
   _left1.set(0);
   _left2.set(0);
   _right1.set(0);
   _right2.set(0);

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

   // Set drive deadband and safety 
   m_drive.setDeadband(0.05);
   m_drive.setSafetyEnabled(false);
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

  @Config(name="Max Drive Output", defaultValueNumeric = 1)
   /**
   * Sets the max output of the drive.  Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  @Log.Graph(name = "Sonar Distance Inches")
  public double getSonarDistanceInches(){
    return m_ultrasonic.getValue()*kValueToInches;
  }
}