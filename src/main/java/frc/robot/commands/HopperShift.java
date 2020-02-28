/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.HopperSubsystem;

public class HopperShift extends CommandBase {
  private final HopperSubsystem m_hopper;

  public HopperShift(HopperSubsystem subsystem) {
    m_hopper = subsystem;
    addRequirements(m_hopper);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    m_hopper.HopperMotor(0.3);
  }

  @Override
  public void end(boolean interrupted) {
    m_hopper.HopperMotor(0);
  }

  @Override
  public boolean isFinished() {
    return !m_hopper.isHighSwitchSet();
  }
}
