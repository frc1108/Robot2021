/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.HopperSubsystem;

public class RaiseHopper extends CommandBase {
  private final HopperSubsystem m_hopper;

  /**
   * Creates a new DriveDistance.
   *
   * @param inches The number of inches the robot will drive
   * @param speed The speed at which the robot will drive
   * @param drive The drive subsystem on which this command will run
   */
  public RaiseHopper(HopperSubsystem subsystem) {
    m_hopper = subsystem;
  }

  @Override
  public void initialize() {
   m_hopper.HopperMotor(0.1);
  }

  @Override
  public void end(boolean interrupted) {
    m_hopper.HopperMotor(0);
  }

  @Override
  public boolean isFinished() {
    return m_hopper.isHighSwitchSet();
  }
}
