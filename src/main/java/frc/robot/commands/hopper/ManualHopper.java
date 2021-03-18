 /*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.hopper;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.HopperSubsystem;

public class ManualHopper extends CommandBase {
  private final HopperSubsystem m_hopper;
  private final DoubleSupplier hopper_spd;
  /**
   */
  public ManualHopper(HopperSubsystem subsystem, DoubleSupplier speed) {
    m_hopper = subsystem;
    hopper_spd = speed;
    addRequirements(m_hopper);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    m_hopper.set(hopper_spd.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
    m_hopper.set(0);
  }

  @Override
  public boolean isFinished() {
   return false;
  }
}
