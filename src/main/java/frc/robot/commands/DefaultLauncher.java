/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.BallLauncher;

/**
 * A command to drive the robot with joystick input
 */
public class DefaultLauncher extends CommandBase {
  private final BallLauncher m_throw;

  /**
   * Creates a new DefaultDrive.
   *
   * @param subsystem The drive subsystem this command wil run on.
   * @param leftMotor Speed for the left motor
   * @param rightMotor Speed for the right motor
   */
  public DefaultLauncher(BallLauncher subsystem) {
    m_throw = subsystem;
    addRequirements(m_throw);
  }

  @Override
  public void initialize() {
    m_throw.startLauncher();
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupt){
    m_throw.stopLauncher();
  }

  @Override
  public boolean isFinished(){
    return false;
  }
}
