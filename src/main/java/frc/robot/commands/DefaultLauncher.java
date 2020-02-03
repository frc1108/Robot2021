/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.BallLauncher;

/**
 * A command to drive the robot with joystick input
 */
public class DefaultLauncher extends CommandBase {
  private final BallLauncher m_throw;
  private final DoubleSupplier m_left;
  private final DoubleSupplier m_right;

  /**
   * Creates a new DefaultDrive.
   *
   * @param subsystem The drive subsystem this command wil run on.
   * @param leftMotor Speed for the left motor
   * @param rightMotor Speed for the right motor
   */
  public DefaultLauncher(BallLauncher subsystem, DoubleSupplier leftMotor, DoubleSupplier rightMotor) {
    m_throw = subsystem;
    m_left = leftMotor;
    m_right = rightMotor;
    addRequirements(m_throw);
  }

  @Override
  public void execute() {
    m_throw.throwBall(m_left.getAsDouble(), m_right.getAsDouble());
  }
}
