/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
//import edu.wpi.first.wpilibj.PWMVictorSPX;
//import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import static frc.robot.Constants.DriveConstants.kEncoderDistancePerPulse;
import static frc.robot.Constants.DriveConstants.kLeftEncoderPorts;
import static frc.robot.Constants.DriveConstants.kLeftEncoderReversed;

import static frc.robot.Constants.DriveConstants.kRightEncoderPorts;
import static frc.robot.Constants.DriveConstants.kRightEncoderReversed;

import static frc.robot.Constants.DriveConstants.CAN_ID_LEFT_DRIVE;
import static frc.robot.Constants.DriveConstants.CAN_ID_RIGHT_DRIVE;

public class DriveSubsystem extends SubsystemBase {

  WPI_TalonSRX _leftTal = new WPI_TalonSRX(CAN_ID_LEFT_DRIVE);
  WPI_VictorSPX _rightVic = new WPI_VictorSPX(CAN_ID_RIGHT_DRIVE);

  // The robot's drive
  private final DifferentialDrive m_drive = new DifferentialDrive(_leftTal, _rightVic);

  // The left-side drive encoder
  private final Encoder m_leftEncoder =
      new Encoder(kLeftEncoderPorts[0], kLeftEncoderPorts[1], kLeftEncoderReversed);

  // The right-side drive encoder
  private final Encoder m_rightEncoder =
      new Encoder(kRightEncoderPorts[0], kRightEncoderPorts[1], kRightEncoderReversed);

  /**
   * Creates a new DriveSubsystem.
   */
  public DriveSubsystem() {
    // Sets the distance per pulse for the encoders
    m_leftEncoder.setDistancePerPulse(kEncoderDistancePerPulse);
    m_rightEncoder.setDistancePerPulse(kEncoderDistancePerPulse);
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(fwd, rot);
  }

  /**
   * Resets the drive encoders to currently read a position of 0.
   */
  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  /**
   * Gets the average distance of the TWO encoders.
   *
   * @return the average of the TWO encoder readings
   */
  public double getAverageEncoderDistance() {
    return (m_leftEncoder.getDistance() + m_rightEncoder.getDistance()) / 2.0;
  }

  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  public Encoder getLeftEncoder() {
    return m_leftEncoder;
  }

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  public Encoder getRightEncoder() {
    return m_rightEncoder;
  }

  /**
   * Sets the max output of the drive.  Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }
}
